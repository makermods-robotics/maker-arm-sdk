"""Make a visual-only loop removal and record geometry needed for the gripper fix.

This does not invent jaw travel, a motor-to-gap calibration, or inertial data.
The two decorative components were identified by inspecting their actual STLs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

LOOP_IDS = (46, 47)
LOOP_MESHES = {f"meshes/part_{i:03d}_solid_{i:03d}.stl" for i in LOOP_IDS}


def build_visual_revision(source, output):
    source, output = Path(source), Path(output)
    if source.resolve() == output.resolve():
        raise ValueError("Write the revision to a separate ZIP")
    with zipfile.ZipFile(source) as archive:
        robot = ET.fromstring(archive.read("robot.urdf"))
        original_inertials = {l.get("name"): ET.tostring(l.find("inertial"))
                              for l in robot.findall("link") if l.find("inertial") is not None}
        original_collisions = [ET.tostring(c) for c in robot.findall("./link/collision")]
        removed = []
        for link in robot.findall("link"):
            for visual in list(link.findall("visual")):
                mesh = visual.find("geometry/mesh")
                if mesh is not None and mesh.get("filename") in LOOP_MESHES:
                    removed.append({"link": link.get("name"), "mesh": mesh.get("filename")})
                    link.remove(visual)
        if {r['mesh'] for r in removed} != LOOP_MESHES or len(removed) != 2:
            raise ValueError("Expected exactly the two identified decorative loop visuals")
        assert original_collisions == [ET.tostring(c) for c in robot.findall("./link/collision")]
        assert original_inertials == {l.get("name"): ET.tostring(l.find("inertial"))
                                     for l in robot.findall("link") if l.find("inertial") is not None}
        report = {
            "revision": "decorative_loops_hidden_visually",
            "source_filename": source.name,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "removed_visuals": removed,
            "collision_and_inertial_properties": "unchanged_real_arm_keeps_loops_user_confirmed",
            "gripper_motion": "not_fixed_yet_requires_measurements_and_linkage_definition",
            "rl_ready": False,
            "notes": ["The two loop STLs remain in meshes because collision references still use them.",
                      "Do not treat visual removal as removal of physical mass or collision geometry.",
                      "Original exporter metadata is archived under source_metadata; it is not revised edit state."]
        }
        ET.indent(robot, space="  ")
        output.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as dest:
            dest.writestr("robot.urdf", ET.tostring(robot, encoding="utf-8", xml_declaration=True))
            for name in archive.namelist():
                if name.startswith("meshes/") and not name.endswith("/"):
                    dest.writestr(name, archive.read(name))
                elif name.endswith(".json"):
                    dest.writestr("source_metadata/" + name, archive.read(name))
            dest.writestr("revision_report.json", json.dumps(report, indent=2))
            dest.writestr("README.txt", "VISUAL-ONLY WORKING REVISION\n\n"
                          "Two decorative loop visuals (part_046 and part_047) are removed.\n"
                          "Their collision geometry and the original link inertia are preserved because\n"
                          "the real arm keeps the loops. Gripper motion is still pending.\n"
                          "This revision is not validated for RL or physics simulation.\n"
                          "See revision_report.json for exact scope and source provenance.\n")
        return report


def geometry_audit(source):
    with zipfile.ZipFile(source) as archive:
        robot = ET.fromstring(archive.read("robot.urdf"))
        link = robot.find("./link[@name='link_007']")
        visual = link.find("visual")
        return {
            "source_sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            "decorative_loop_parts_confirmed": ["part_046", "part_047"],
            "candidate_jaw_rigid_groups": {
                "cad_positive_z": {"carriage": "part_033", "mount": "part_034", "finger": "part_045"},
                "cad_negative_z": {"carriage": "part_040", "mount": "part_041", "finger": "part_044"}
            },
            "stationary_rail_candidate": "part_027",
            "drive_crank_candidate": "part_023",
            "connecting_rods": ["part_030", "part_037"],
            "slide_direction_cad": [0, 0, 1],
            "slide_direction_link_007_approximately": [0, 1, 0],
            "original_mesh_origin_in_link_007": dict(visual.find("origin").attrib),
            "kinematic_plan": "Two opposing prismatic jaw joints; one independent gap coordinate. The crank and rods require a closed-loop model or a documented simplified representation.",
            "status": "geometry_identified_not_yet_verified_against_real_gripper",
            "measurements_needed": {
                "closed_inside_jaw_gap_mm": None,
                "open_inside_jaw_gap_mm": None,
                "maker_motor_angle_to_gap_samples": [],
                "maker_encoder_angle_unit": None,
                "jaw_motion_symmetric_and_straight": None,
                "decorative_loops_physically_removed": None,
                "moving_positive_jaw_assembly_mass_kg": None,
                "moving_negative_jaw_assembly_mass_kg": None,
                "stationary_gripper_assembly_mass_kg": None,
                "gripping_force_newtons": None,
                "jaw_speed_m_per_s": None
            }
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    report = build_visual_revision(a.source, a.output)
    audit_path = a.output.with_name("gripper_geometry_and_measurements.json")
    audit_path.write_text(json.dumps(geometry_audit(a.source), indent=2) + "\n")
    print(json.dumps({"output": str(a.output), "audit": str(audit_path), "removed": report["removed_visuals"]}, indent=2))


if __name__ == "__main__":
    main()
