import struct

from maker_arm import protocol as p


def feedback_frame(motor_id, pos=0.0, vel=0.0, tau=0.0, temp=30.0, fault=0, mode=2, params=None):
    pr = params or p.RS00
    cid = (p.COMM_FEEDBACK << 24) | (mode << 22) | (fault << 16) | (motor_id << 8) | p.HOST_CAN_ID
    data = struct.pack(">HHHH",
                       p.float_to_u16(pos, pr.p_min, pr.p_max),
                       p.float_to_u16(vel, pr.v_min, pr.v_max),
                       p.float_to_u16(tau, pr.t_min, pr.t_max),
                       int(temp * 10))
    return cid, data
