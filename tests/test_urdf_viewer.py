import json
import struct
import zipfile
from types import SimpleNamespace

import pytest

from tools.diagnostics.urdf_viewer.server import StarReader, describe_model
from tools.diagnostics.urdf_viewer.star_serial import angle_request, parse_angle


def packet(servo, angle_tenths, code=16):
    body = bytes([5, 28, code, 7, servo]) + struct.pack('<i', angle_tenths) + b'\0\0'
    return body + bytes([sum(body) & 255])


def test_wire_request_is_only_a_multi_turn_angle_query():
    assert angle_request(0) == bytes([0x12, 0x4c, 16, 1, 0, 0x6f])
    assert all(angle_request(i)[2:4] == bytes([16, 1]) for i in range(7))
    with pytest.raises(ValueError):
        angle_request(254)


@pytest.mark.parametrize('value', [0, 1, -1, -2708, 7250])
def test_sensor_zero_negative_and_multi_turn_angles_are_real_readings(value):
    sample = parse_angle(bytearray(packet(2, value)), 2)
    assert sample.angle_deg == value / 10
    assert sample.reliable


def test_fragmented_packet_noise_wrong_id_and_checksum():
    response = packet(3, -125)
    buffer = bytearray(b'noise' + response[:8])
    assert parse_angle(buffer, 3) is None
    buffer.extend(response[8:])
    assert parse_angle(buffer, 3).angle_deg == -12.5
    corrupt = bytearray(packet(3, 0)); corrupt[-1] ^= 1
    assert parse_angle(bytearray(corrupt), 3) is None
    assert parse_angle(bytearray(packet(2, 10)), 3) is None
    assert parse_angle(bytearray(packet(3, 10, code=8)), 3) is None
    # A bad frame must not hide a valid response following it.
    assert parse_angle(corrupt + bytearray(response), 3).angle_deg == -12.5


def test_stale_data_and_partial_readings_do_not_create_a_rest_reference():
    reader = StarReader('test-port')
    reader.ingest({0: SimpleNamespace(angle_deg=0, reliable=True)}, 10)
    assert reader.snapshot(10.1)['fresh_count'] == 1
    assert reader.initial_reference is None
    reader.ingest({i: SimpleNamespace(angle_deg=float(i), reliable=True) for i in range(1, 7)}, 11)
    assert reader.initial_reference is None  # first joint has already expired
    reader.ingest({0: SimpleNamespace(angle_deg=0, reliable=True)}, 11.1)
    reference = json.dumps(reader.initial_reference)
    assert reader.snapshot(11.2)['all_fresh']
    reader.ingest({0: SimpleNamespace(angle_deg=999, reliable=False)}, 11.3)
    assert reader.snapshot(11.3)['readings'][0]['angle_deg'] == 0
    reader.ingest({0: SimpleNamespace(angle_deg=float('nan'), reliable=True)}, 11.4)
    assert reader.snapshot(11.7)['fresh_count'] == 0
    reader.ingest({0: SimpleNamespace(angle_deg=10, reliable=True)}, 12)
    assert json.dumps(reader.initial_reference) == reference


def make_model(tmp_path, missing_mesh=False, cycle=False):
    links = ['base_link'] + [f'link_{i:03d}' for i in range(2,8)]
    xml = '<robot name="test">' + ''.join(f'<link name="{n}"><visual><geometry><mesh filename="mesh.stl"/></geometry></visual></link>' for n in links)
    for i in range(6):
        parent = links[i] if not (cycle and i == 0) else links[-1]
        xml += f'<joint name="link_{i+2:03d}_joint" type="revolute"><parent link="{parent}"/><child link="{links[i+1]}"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
    xml += '</robot>'
    path=tmp_path/'robot.zip'
    with zipfile.ZipFile(path, 'w') as z:
        z.writestr('robot.urdf', xml)
        if not missing_mesh: z.writestr('mesh.stl', b'test fixture')
    return path


def test_model_audit_finds_exact_six_joint_structure(tmp_path):
    model=describe_model(make_model(tmp_path))
    assert model['links']==7
    assert len(model['joints'])==6
    assert not model['gripper_joint_present']
    assert len(model['sha256'])==64


def test_model_audit_rejects_missing_assets_and_cycles(tmp_path):
    with pytest.raises(ValueError, match='mesh'):
        describe_model(make_model(tmp_path, missing_mesh=True))
    with pytest.raises(ValueError, match='cyclic'):
        describe_model(make_model(tmp_path, cycle=True))
