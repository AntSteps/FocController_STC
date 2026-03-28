#!/usr/bin/env python3
"""FOC Controller 纯协议编解码层，可复用于 PC 和 MicroPython。"""

import struct


ACK_REPLY_HEAD = 0x0621
STATE_REPLY_HEAD = 0x0721
PARAM_VALUE_REPLY_HEAD = 0x0821
PARAM_SNAPSHOT_REPLY_HEAD = 0x0921
CALIB_STATUS_REPLY_HEAD = 0x0A21

CTRL_CMD_HEAD = 0x1145
PARAM_WRITE_CMD_HEAD = 0x1245
QUERY_CMD_HEAD = 0x1345
CALIB_CMD_HEAD = 0x1445

QUERY_GET_STATE = 0x01
QUERY_GET_PARAM = 0x02
QUERY_GET_PARAM_SNAPSHOT = 0x03
QUERY_GET_CALIB_STATUS = 0x04

CALIB_CMD_START_AUTO = 0x01

REQUEST_KIND_CTRL = 1
REQUEST_KIND_PARAM_WRITE = 2
REQUEST_KIND_QUERY = 3
REQUEST_KIND_CALIB = 4

CTRL_MODE_STOP = 0
CTRL_MODE_SPEED = 1

RESULT_OK = 0
RESULT_BUSY = 1
RESULT_UNSUPPORTED = 2
RESULT_INVALID_PARAM = 3
RESULT_INVALID_VALUE = 4
RESULT_EEPROM_ERROR = 5
RESULT_CALIB_FAILED = 6
RESULT_INTERNAL_ERROR = 7

CALIB_STATE_IDLE = 0
CALIB_STATE_RUNNING = 1
CALIB_STATE_SUCCESS = 2
CALIB_STATE_FAIL_ENCODER = 3
CALIB_STATE_FAIL_BUSY = 4
CALIB_STATE_FAIL_INTERNAL = 5

STATUS_ENCODER_FAULT = 0x01
STATUS_MOTOR_FAULT = 0x02
STATUS_ENC_MAG_MASK = 0x0C
STATUS_ENC_MAG_SHIFT = 2
STATUS_RESET_LOW_VOLTAGE = 0x10
STATUS_DRIVER_NFAULT = 0x20
STATUS_ENC_DATA_GOOD = 0x40
STATUS_PARAM_FROM_EEPROM = 0x80

CTRL_CMD_FRAME_FORMAT = ">HBBfffBB"
PARAM_WRITE_CMD_FRAME_FORMAT = ">HBBfBB"
QUERY_CMD_FRAME_FORMAT = ">HBBBBB"
CALIB_CMD_FRAME_FORMAT = ">HBBBBB"

ACK_REPLY_FRAME_FORMAT = ">HBBBBB"
STATE_REPLY_FRAME_FORMAT = ">HBBfffffBBB"
PARAM_VALUE_REPLY_FRAME_FORMAT = ">HBBfBB"
PARAM_SNAPSHOT_REPLY_FRAME_FORMAT = ">HBBhffffffffffffBfffhhhfffffBB"
CALIB_STATUS_REPLY_FRAME_FORMAT = ">HBBBBB"

CTRL_CMD_FRAME_SIZE = struct.calcsize(CTRL_CMD_FRAME_FORMAT)
PARAM_WRITE_CMD_FRAME_SIZE = struct.calcsize(PARAM_WRITE_CMD_FRAME_FORMAT)
QUERY_CMD_FRAME_SIZE = struct.calcsize(QUERY_CMD_FRAME_FORMAT)
CALIB_CMD_FRAME_SIZE = struct.calcsize(CALIB_CMD_FRAME_FORMAT)

ACK_REPLY_FRAME_SIZE = struct.calcsize(ACK_REPLY_FRAME_FORMAT)
STATE_REPLY_FRAME_SIZE = struct.calcsize(STATE_REPLY_FRAME_FORMAT)
PARAM_VALUE_REPLY_FRAME_SIZE = struct.calcsize(PARAM_VALUE_REPLY_FRAME_FORMAT)
PARAM_SNAPSHOT_REPLY_FRAME_SIZE = struct.calcsize(PARAM_SNAPSHOT_REPLY_FRAME_FORMAT)
CALIB_STATUS_REPLY_FRAME_SIZE = struct.calcsize(CALIB_STATUS_REPLY_FRAME_FORMAT)

DEFAULT_BAUDRATE = 1_000_000

MAG_FIELD_LABELS = {
    0: "磁场正常",
    1: "磁场过强",
    2: "磁场过弱",
    3: "磁场无效",
}

REPLY_FRAME_SPECS = {
    ACK_REPLY_HEAD: (ACK_REPLY_FRAME_SIZE, "ACK"),
    STATE_REPLY_HEAD: (STATE_REPLY_FRAME_SIZE, "STATE"),
    PARAM_VALUE_REPLY_HEAD: (PARAM_VALUE_REPLY_FRAME_SIZE, "PARAM"),
    PARAM_SNAPSHOT_REPLY_HEAD: (PARAM_SNAPSHOT_REPLY_FRAME_SIZE, "SNAPSHOT"),
    CALIB_STATUS_REPLY_HEAD: (CALIB_STATUS_REPLY_FRAME_SIZE, "CALIB"),
}

CONTROL_MODE_SUMMARY_LABELS = {
    CTRL_MODE_STOP: "停止/休眠",
    CTRL_MODE_SPEED: "速度模式",
}

QUERY_SUMMARY_LABELS = {
    QUERY_GET_STATE: "读取状态",
    QUERY_GET_PARAM: "读取参数",
    QUERY_GET_PARAM_SNAPSHOT: "读取参数快照",
    QUERY_GET_CALIB_STATUS: "读取校准状态",
}

CALIB_CMD_SUMMARY_LABELS = {
    CALIB_CMD_START_AUTO: "开始自动校准",
}


def crc8_check(data, has_crc=0):
    crc = 0x00
    length = len(data) - has_crc
    index = 0
    while index < length:
        crc ^= data[index]
        bit_index = 0
        while bit_index < 8:
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            bit_index += 1
        index += 1
    return crc


def format_hex(data):
    return " ".join("%02X" % byte for byte in data)


def parse_hex_string(text):
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = [token for token in cleaned.split() if token]
    if not tokens:
        raise ValueError("请输入十六进制字节，例如 13 45 01 01 00 01")
    return bytes(int(token, 16) & 0xFF for token in tokens)


def parse_status(status):
    details = []
    mag_field = (status & STATUS_ENC_MAG_MASK) >> STATUS_ENC_MAG_SHIFT

    if status & STATUS_ENCODER_FAULT:
        details.append("编码器异常")
    else:
        details.append("编码器正常")

    if status & STATUS_MOTOR_FAULT:
        details.append("电机异常")

    details.append(MAG_FIELD_LABELS.get(mag_field, "磁场未知"))

    if status & STATUS_ENC_DATA_GOOD:
        details.append("数据有效")
    else:
        details.append("数据无效")

    if status & STATUS_RESET_LOW_VOLTAGE:
        details.append("低压复位")

    if status & STATUS_DRIVER_NFAULT:
        details.append("驱动故障(nFAULT)")

    if status & STATUS_PARAM_FROM_EEPROM:
        details.append("参数来自EEPROM")
    else:
        details.append("参数未从EEPROM加载")

    return ", ".join(details)


def pack_frame(fmt, *values):
    payload = struct.pack(fmt, *values)
    crc = crc8_check(payload, has_crc=1)
    return payload[:-1] + bytes([crc])


def unpack_frame(fmt, data, expected_head, frame_name):
    if len(data) != struct.calcsize(fmt):
        raise ValueError("%s 长度错误: %d" % (frame_name, len(data)))
    values = struct.unpack(fmt, data)
    head = int(values[0])
    if head != expected_head:
        raise ValueError("%s 帧头错误: 0x%04X" % (frame_name, head))
    calc_crc = crc8_check(data, has_crc=1)
    recv_crc = int(values[-1])
    if calc_crc != recv_crc:
        raise ValueError(
            "%s CRC 错误: 计算 0x%02X, 接收 0x%02X" % (frame_name, calc_crc, recv_crc)
        )
    return values


def build_ctrl_cmd_frame(motor_id, ctrl_mode, target, ff0, ff1, seq):
    return pack_frame(
        CTRL_CMD_FRAME_FORMAT,
        CTRL_CMD_HEAD,
        motor_id & 0xFF,
        ctrl_mode & 0xFF,
        float(target),
        float(ff0),
        float(ff1),
        seq & 0xFF,
        0,
    )


def build_param_write_frame(motor_id, param_id, value, seq):
    return pack_frame(
        PARAM_WRITE_CMD_FRAME_FORMAT,
        PARAM_WRITE_CMD_HEAD,
        motor_id & 0xFF,
        param_id & 0xFF,
        float(value),
        seq & 0xFF,
        0,
    )


def build_query_frame(motor_id, query_type, arg0, seq):
    return pack_frame(
        QUERY_CMD_FRAME_FORMAT,
        QUERY_CMD_HEAD,
        motor_id & 0xFF,
        query_type & 0xFF,
        arg0 & 0xFF,
        seq & 0xFF,
        0,
    )


def build_calib_frame(motor_id, calib_cmd, arg0, seq):
    return pack_frame(
        CALIB_CMD_FRAME_FORMAT,
        CALIB_CMD_HEAD,
        motor_id & 0xFF,
        calib_cmd & 0xFF,
        arg0 & 0xFF,
        seq & 0xFF,
        0,
    )


def decode_ack_reply(data):
    values = unpack_frame(ACK_REPLY_FRAME_FORMAT, data, ACK_REPLY_HEAD, "AckReply")
    return {
        "kind": "ack",
        "head": ACK_REPLY_HEAD,
        "motor_id": int(values[1]),
        "cmd_type": int(values[2]),
        "result_code": int(values[3]),
        "seq": int(values[4]),
        "crc8": int(values[5]),
    }


def decode_state_reply(data):
    values = unpack_frame(STATE_REPLY_FRAME_FORMAT, data, STATE_REPLY_HEAD, "StateReply")
    return {
        "kind": "state",
        "head": STATE_REPLY_HEAD,
        "motor_id": int(values[1]),
        "control_mode": int(values[2]),
        "target": float(values[3]),
        "velocity": float(values[4]),
        "output": float(values[5]),
        "position": float(values[6]),
        "voltage": float(values[7]),
        "health_status": int(values[8]),
        "seq": int(values[9]),
        "crc8": int(values[10]),
    }


def decode_param_value_reply(data):
    values = unpack_frame(
        PARAM_VALUE_REPLY_FRAME_FORMAT,
        data,
        PARAM_VALUE_REPLY_HEAD,
        "ParamValueReply",
    )
    return {
        "kind": "param_value",
        "head": PARAM_VALUE_REPLY_HEAD,
        "motor_id": int(values[1]),
        "param_id": int(values[2]),
        "value": float(values[3]),
        "seq": int(values[4]),
        "crc8": int(values[5]),
    }


def decode_param_snapshot_reply(data):
    values = unpack_frame(
        PARAM_SNAPSHOT_REPLY_FRAME_FORMAT,
        data,
        PARAM_SNAPSHOT_REPLY_HEAD,
        "ParamSnapshotReply",
    )
    return {
        "kind": "param_snapshot",
        "head": PARAM_SNAPSHOT_REPLY_HEAD,
        "motor_id": int(values[1]),
        "snapshot_version": int(values[2]),
        "snapshot": {
            "motor_pole_pairs": int(values[3]),
            "motor_phase_resistance": float(values[4]),
            "motor_phase_inductance": float(values[5]),
            "cur_kp": float(values[6]),
            "cur_ki": float(values[7]),
            "speed_kp": float(values[8]),
            "speed_ki": float(values[9]),
            "speed_out_max": float(values[10]),
            "speed_out_min": float(values[11]),
            "speed_lpf_time_constant_ms": float(values[12]),
            "ff_velocity_gain": float(values[13]),
            "ff_acceleration_gain": float(values[14]),
            "ff_friction_compensation": float(values[15]),
            "ff_enable": int(values[16]),
            "speed_sample_freq_hz": float(values[17]),
            "current_limit": float(values[18]),
            "velocity_limit": float(values[19]),
            "encoder_offset": int(values[20]),
            "calib_pole_pairs": int(values[21]),
            "calib_dir": int(values[22]),
            "calib_electrical_offset_rad": float(values[23]),
            "calib_harmonic_amp_1": float(values[24]),
            "calib_harmonic_phase_1": float(values[25]),
            "calib_harmonic_amp_2": float(values[26]),
            "calib_harmonic_phase_2": float(values[27]),
        },
        "seq": int(values[28]),
        "crc8": int(values[29]),
    }


def decode_calib_status_reply(data):
    values = unpack_frame(
        CALIB_STATUS_REPLY_FRAME_FORMAT,
        data,
        CALIB_STATUS_REPLY_HEAD,
        "CalibStatusReply",
    )
    return {
        "kind": "calib_status",
        "head": CALIB_STATUS_REPLY_HEAD,
        "motor_id": int(values[1]),
        "calib_state": int(values[2]),
        "progress": int(values[3]),
        "seq": int(values[4]),
        "crc8": int(values[5]),
    }


def decode_reply_frame(data):
    if len(data) < 2:
        raise ValueError("回复帧长度不足")

    head = (data[0] << 8) | data[1]
    if head == ACK_REPLY_HEAD:
        return decode_ack_reply(data)
    if head == STATE_REPLY_HEAD:
        return decode_state_reply(data)
    if head == PARAM_VALUE_REPLY_HEAD:
        return decode_param_value_reply(data)
    if head == PARAM_SNAPSHOT_REPLY_HEAD:
        return decode_param_snapshot_reply(data)
    if head == CALIB_STATUS_REPLY_HEAD:
        return decode_calib_status_reply(data)
    raise ValueError("未知回复帧头: 0x%04X" % head)
