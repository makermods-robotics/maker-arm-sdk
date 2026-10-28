"""Build a reduced two-slider gripper from the designer's Maker Arm export.

The original mesh coordinates and closed pose are preserved. The closed-loop
drive is omitted from geometry; its inertia remains lumped into the wrist.
Mass redistribution uses the source export's assumed uniform density, not a
claim that the assemblies have been weighed.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

GAP_M = 0.104825
TRAVEL_M = GAP_M / 2
JAW_PARTS = {
    "gripper_left": {31, 32, 33, 34, 43, 45, 47},
    "gripper_right": {38, 39, 40, 41, 44, 46},
}
LOOPS = {46, 47}
# The crank, connecting rods, and hardware rigidly carried by the crank.
DRIVE_PARTS = {23, 24, 25, 28, 29, 30, 35, 36, 37, 42}


def mesh_id(element):
    return int(element.find("geometry/mesh").get("filename").split("part_")[1][:3])


def rotation(rpy):
    r, p, y = map(float, rpy.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def triangles(blob):
    count = struct.unpack_from("<I", blob, 80)[0]
    if len(blob) != 84 + count * 50:
        raise ValueError("Expected the designer's binary STL format")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(blob, dtype=dtype, offset=84)["vertices"].astype(float) * .001


def volume_moments(vertices):
    """Signed tetrahedral volume, first moment, and second moment of volume."""
    a, b, c = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    v = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    sums = a+b+c
    second = np.einsum("ni,nj->nij", sums, sums)
    for points in (a, b, c):
        second += np.einsum("ni,nj->nij", points, points)
    total = v.sum()
    if abs(total) < 1e-15:
        raise ValueError("Mesh has zero signed volume")
    sign = 1 if total > 0 else -1
    return sign*total, sign*np.einsum("n,ni->i", v, sums)/4, sign*np.einsum("n,nij->ij", v, second)/20


def inertia_origin(second):
    return np.trace(second)*np.eye(3) - second


def inertial_values(node):
    m = float(node.find("mass").get("value"))
    origin = node.find("origin")
    com = np.array(list(map(float, origin.get("xyz", "0 0 0").split())))
    rot = rotation(origin.get("rpy", "0 0 0"))
    i = node.find("inertia").attrib
    tensor = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                       [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                       [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
    return m, m*com, rot@tensor@rot.T + m*(np.dot(com, com)*np.eye(3)-np.outer(com, com))


def put_inertia(link, mass, first, about_origin):
    com = first/mass
    tensor = about_origin - mass*(np.dot(com, com)*np.eye(3)-np.outer(com, com))
    tensor = (tensor + tensor.T)/2
    eig = np.linalg.eigvalsh(tensor)
    if mass <= 0 or min(eig) <= 0 or max(eig) > sum(eig)-max(eig)+1e-9:
        raise ValueError(f"Invalid redistributed inertia for {link.get('name')}: {eig}")
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    node = ET.SubElement(link, "inertial")
    ET.SubElement(node, "origin", xyz=" ".join(f"{v:.12g}" for v in com), rpy="0 0 0")
    ET.SubElement(node, "mass", value=f"{mass:.12g}")
    ET.SubElement(node, "inertia", **{name: f"{tensor[i,j]:.12g}" for name, i, j in
                                   [("ixx",0,0),("ixy",0,1),("ixz",0,2),("iyy",1,1),("iyz",1,2),("izz",2,2)]})
    return {"mass_kg": mass, "com_m": com.tolist(), "principal_inertias_kg_m2": eig.tolist()}


def build(source, output, endpoints):
    source, output = Path(source), Path(output)
    if source.resolve() == output.resolve():
        raise ValueError("Revision must be separate from source")
    measurements = json.loads(Path(endpoints).read_text())
    with zipfile.ZipFile(source) as archive:
        robot = ET.fromstring(archive.read("robot.urdf"))
        robot.set("name", "maker_arm_gripper_v1")
        wrist = robot.find("./link[@name='link_007']")
        original_joints = [ET.tostring(j) for j in robot.findall("joint")]
        original = inertial_values(wrist.find("inertial"))
        origin = wrist.find("visual/origin")
        rot = rotation(origin.get("rpy"))
        offset = np.array(list(map(float, origin.get("xyz").split())))
        moments = {}
        for visual in wrist.findall("visual"):
            part = mesh_id(visual)
            verts = triangles(archive.read(visual.find("geometry/mesh").get("filename")))
            moments[part] = volume_moments(verts@rot.T + offset)
        density = original[0]/sum(x[0] for x in moments.values())
        residual = [deepcopy(x) for x in original]
        inertias = {}
        removed = {"visual_loops": [], "internal_drive_visuals_and_collisions": []}
        for name, parts in JAW_PARTS.items():
            jaw = ET.SubElement(robot, "link", name=name)
            for tag in ("visual", "collision"):
                for element in list(wrist.findall(tag)):
                    part = mesh_id(element)
                    if part not in parts:
                        continue
                    wrist.remove(element)
                    if tag == "visual" and part in LOOPS:
                        removed["visual_loops"].append(part)
                    else:
                        jaw.append(element)
            mass = density*sum(moments[i][0] for i in parts)
            first = density*sum((moments[i][1] for i in parts), np.zeros(3))
            io = density*sum((inertia_origin(moments[i][2]) for i in parts), np.zeros((3,3)))
            inertias[name] = put_inertia(jaw, mass, first, io)
            for i, value in enumerate((mass, first, io)):
                residual[i] -= value
            joint = ET.SubElement(robot, "joint", name=name+"_joint", type="prismatic")
            ET.SubElement(joint, "parent", link="link_007")
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
            axis = rot[:, 2] * (1 if name.endswith("left") else -1)
            ET.SubElement(joint, "axis", xyz=" ".join(f"{v:.12g}" for v in axis))
            # These actuator bounds are declared placeholders in the report.
            ET.SubElement(joint, "limit", lower="0", upper=str(TRAVEL_M), effort="20", velocity="0.1")
            if name.endswith("right"):
                ET.SubElement(joint, "mimic", joint="gripper_left_joint", multiplier="1", offset="0")
        for tag in ("visual", "collision"):
            for element in list(wrist.findall(tag)):
                if mesh_id(element) in DRIVE_PARTS:
                    removed["internal_drive_visuals_and_collisions"].append({"part": mesh_id(element), "kind": tag})
                    wrist.remove(element)
        inertias["link_007"] = put_inertia(wrist, *residual)
        assert [ET.tostring(j) for j in robot.findall("joint")[:6]] == original_joints
        # Grasp frame at the area centroid of the inner contact face in CAD zero.
        verts = triangles(archive.read("meshes/part_045_solid_045.stl"))
        faces = verts[np.max(np.abs(verts[:,:,2]), axis=1) < 1e-7]
        areas = np.linalg.norm(np.cross(faces[:,1]-faces[:,0], faces[:,2]-faces[:,0]), axis=1)/2
        contact_center = np.average(faces.mean(axis=1), axis=0, weights=areas)
        contact_center[2] = 0
        center = rot@contact_center + offset
        ET.SubElement(robot, "link", name="grasp_center")
        tool = ET.SubElement(robot, "joint", name="grasp_center_joint", type="fixed")
        ET.SubElement(tool, "parent", link="link_007")
        ET.SubElement(tool, "child", link="grasp_center")
        ET.SubElement(tool, "origin", xyz=" ".join(f"{x:.12g}" for x in center), rpy="0 0 0")
        report = {
            "schema": 1, "revision": "two_symmetric_sliding_jaws_visual_loops_removed",
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "kinematics": {"independent_arm_joints": 6, "independent_gripper_joints": 1,
                           "driver_joint": "gripper_left_joint", "follower_joint": "gripper_right_joint",
                           "gap_m": [0, GAP_M], "per_jaw_travel_m": TRAVEL_M,
                           "mapping": "gap_m = 2 * gripper_left_joint (metres)",
                           "source": "user-reported gap and equal straight jaw travel",
                           "grasp_frame": "CAD inner-face area centroid; midpoint between symmetric fingers"},
            "jaw_part_assignments": {k: sorted(v) for k,v in JAW_PARTS.items()},
            "removed_geometry": removed,
            "loops": "Visuals removed; real-arm loops retained as collisions and inertia on their respective moving jaws",
            "inertias": inertias,
            "inertia_basis": "Moving assemblies use source terminal-link uniform density from mesh volumes; subtract their mass and full inertia from original wrist at CAD zero. Total mass, COM, inertia at closed are preserved. Internal drive inertia stays lumped into wrist.",
            "source_terminal_density_kg_m3": density,
            "motor_calibration": {"protocol": "MIT", "motor_id": 7,
                                  "closed_encoder_rad": measurements["closed_encoder_rad"],
                                  "commanded_open_encoder_rad": measurements["open_encoder_rad"],
                                  "model": "two-endpoint linear interpolation for visual preview only",
                                  "full_open_gap_at_encoder_verified": False,
                                  "note": "Open capture reached pre-existing safe limit with mechanical-stop backoff; 104.825 mm gap at this encoder was not physically remeasured. Intermediate linkage nonlinearity unmeasured."},
            "rl_ready": False,
            "remaining_validation": [
                "Review CAD part assignments and actual Maker joint axes/zero poses against real arm",
                "Weigh jaw assemblies and fixed wrist; source density and other link inertias unverified",
                "Gripper effort=20 N and velocity=0.1 m/s are explicit placeholders, not measured specifications",
                "Arm joint effort/velocity/limits inherited unverified from designer",
                "Internal crank-and-rod drive omitted from visual/collision geometry; wrist inertia lumps its mass",
                "Verify simulator mimic support, mesh collision decomposition, friction, backlash and self-collision exclusions",
                "Source omitted CAD part_003; purpose and ownership still unverified",
                "Validate actual full-open motor-to-gap relationship and intermediate points before physical control"],
        }
        ET.indent(robot, space="  ")
        output.parent.mkdir(parents=True, exist_ok=True)
        xml = ET.tostring(robot, encoding="utf-8", xml_declaration=True)
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as dest:
            dest.writestr("robot.urdf", xml)
            for name in archive.namelist():
                if name.startswith("meshes/") and not name.endswith("/"):
                    dest.writestr(name, archive.read(name))
                elif name.endswith(".json"):
                    dest.writestr("source_metadata/"+name, archive.read(name))
            dest.writestr("revision_report.json", json.dumps(report, indent=2)+"\n")
            dest.writestr("gripper_mit_endpoints.json", Path(endpoints).read_bytes())
            dest.writestr("README.md", "# Maker Arm gripper inspection revision\n\n"
                          "Load robot.urdf with meshes/ alongside it. Six arm joints plus one independent gripper DOF.\n"
                          "Set gripper_left_joint in metres between 0 and 0.0524125; gripper_right_joint mimics it.\n"
                          "Axes oppose each other, so total jaw gap is twice the driver coordinate: 0–104.825 mm.\n"
                          "The loop visuals are removed; their collision shapes and mass remain on the moving jaws.\n\n"
                          "This is a reduced mechanism for visual/kinematic inspection. Internal crank/rods are omitted;\n"
                          "their inertia is lumped into the wrist. Mass was redistributed from the original CAD estimate.\n"
                          "Gripper 20 N / 0.1 m/s limits are placeholders. This model is not validated for RL dynamics.\n"
                          "See revision_report.json for provenance, assumptions and required physical verification.\n"
                          "Motor endpoint interpolation is a preview approximation, not a calibrated transmission.\n")
        output.with_suffix(".report.json").write_text(json.dumps(report, indent=2)+"\n")
        return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--endpoints", required=True)
    a = ap.parse_args()
    report = build(a.source, a.output, a.endpoints)
    print(json.dumps({"output": a.output, "kinematics": report["kinematics"], "inertias": report["inertias"]}, indent=2))


if __name__ == "__main__":
    main()
