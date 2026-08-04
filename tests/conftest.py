import struct

from maker_arm import protocol as p


def feedback_frame(motor_id, pos=0.0, vel=0.0, tau=0.0, temp=30.0, fault=0, mode=2):
    cid = (p.COMM_FEEDBACK << 24) | (mode << 22) | (fault << 16) | (motor_id << 8) | p.HOST_CAN_ID
    data = struct.pack(">HHHH",
                       p.float_to_u16(pos, p.P_MIN, p.P_MAX),
                       p.float_to_u16(vel, p.V_MIN, p.V_MAX),
                       p.float_to_u16(tau, p.T_MIN, p.T_MAX),
                       int(temp * 10))
    return cid, data
