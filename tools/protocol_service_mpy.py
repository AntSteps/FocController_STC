#!/usr/bin/env python3
"""FOC Controller MicroPython 同步协议服务层。"""

import time

from protocol_codec import (
    ACK_REPLY_HEAD,
    CALIB_CMD_FRAME_SIZE,
    CALIB_CMD_HEAD,
    CALIB_CMD_START_AUTO,
    CALIB_STATUS_REPLY_HEAD,
    CTRL_CMD_FRAME_SIZE,
    CTRL_CMD_HEAD,
    CTRL_MODE_SPEED,
    CTRL_MODE_STOP,
    DEFAULT_BAUDRATE,
    PARAM_SNAPSHOT_REPLY_HEAD,
    PARAM_VALUE_REPLY_HEAD,
    PARAM_WRITE_CMD_FRAME_SIZE,
    PARAM_WRITE_CMD_HEAD,
    QUERY_CMD_FRAME_SIZE,
    QUERY_CMD_HEAD,
    QUERY_GET_CALIB_STATUS,
    QUERY_GET_PARAM,
    QUERY_GET_PARAM_SNAPSHOT,
    QUERY_GET_STATE,
    REPLY_FRAME_SPECS,
    STATE_REPLY_HEAD,
    build_calib_frame,
    build_ctrl_cmd_frame,
    build_param_write_frame,
    build_query_frame,
    decode_reply_frame,
)


REQUEST_TIMEOUT_MS = 1500
INTER_FRAME_GAP_MS = 3


class ProtocolTimeoutError(Exception):
    pass


class ProtocolDecodeError(Exception):
    pass


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(now, start):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, start)
    return now - start


def _sleep_ms(duration_ms):
    if duration_ms <= 0:
        return
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(duration_ms)
        return
    time.sleep(duration_ms / 1000.0)


class MicroPythonProtocolService:
    """适用于 UART/RS485 半双工场景的同步协议服务层。"""

    def __init__(
        self,
        uart,
        baudrate=DEFAULT_BAUDRATE,
        request_timeout_ms=REQUEST_TIMEOUT_MS,
        inter_frame_gap_ms=INTER_FRAME_GAP_MS,
        auto_seq=True,
    ):
        self.uart = uart
        self.baudrate = int(baudrate)
        self.request_timeout_ms = int(request_timeout_ms)
        self.inter_frame_gap_ms = int(inter_frame_gap_ms)
        self.auto_seq = bool(auto_seq)
        self.sequence_counter = 0
        self._rx_buffer = bytearray()

    def next_seq(self):
        seq = self.sequence_counter & 0xFF
        self.sequence_counter = (self.sequence_counter + 1) & 0xFF
        return seq

    def set_speed(self, motor_id, speed, load_ff=0.0, seq=None):
        return self.send_control(
            motor_id=motor_id,
            ctrl_mode=CTRL_MODE_SPEED,
            target=speed,
            load_ff=load_ff,
            seq=seq,
        )

    def stop(self, motor_id, seq=None):
        return self.send_control(
            motor_id=motor_id,
            ctrl_mode=CTRL_MODE_STOP,
            target=0.0,
            load_ff=0.0,
            seq=seq,
        )

    def send_control(self, motor_id, ctrl_mode, target, load_ff=0.0, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_ctrl_cmd_frame(motor_id, ctrl_mode, target, load_ff, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(ACK_REPLY_HEAD,),
        )

    def write_param(self, motor_id, param_id, value, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_param_write_frame(motor_id, param_id, value, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(ACK_REPLY_HEAD,),
        )

    def get_state(self, motor_id, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_query_frame(motor_id, QUERY_GET_STATE, 0, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(STATE_REPLY_HEAD,),
        )

    def get_param(self, motor_id, param_id, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_query_frame(motor_id, QUERY_GET_PARAM, param_id, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(PARAM_VALUE_REPLY_HEAD,),
        )

    def get_snapshot(self, motor_id, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_query_frame(motor_id, QUERY_GET_PARAM_SNAPSHOT, 0, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(PARAM_SNAPSHOT_REPLY_HEAD,),
        )

    def get_calibration_status(self, motor_id, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_query_frame(motor_id, QUERY_GET_CALIB_STATUS, 0, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(CALIB_STATUS_REPLY_HEAD,),
        )

    def start_calibration(self, motor_id, seq=None):
        seq = self._resolve_seq(seq)
        frame = build_calib_frame(motor_id, CALIB_CMD_START_AUTO, 0, seq)
        return self._request(
            frame,
            expected_seq=seq,
            expected_motor_id=motor_id,
            expected_heads=(ACK_REPLY_HEAD,),
        )

    def send_raw(self, payload, expected_seq=None, expected_motor_id=None, expected_heads=None):
        return self._request(
            payload=payload,
            expected_seq=expected_seq,
            expected_motor_id=expected_motor_id,
            expected_heads=expected_heads,
        )

    def read_reply(self, timeout_ms=None, expected_seq=None, expected_motor_id=None, expected_heads=None):
        timeout_ms = self.request_timeout_ms if timeout_ms is None else int(timeout_ms)
        deadline = _ticks_ms()

        while _ticks_diff(_ticks_ms(), deadline) < timeout_ms:
            reply = self._try_parse_reply()
            if reply is not None:
                if self._matches(reply, expected_seq, expected_motor_id, expected_heads):
                    return reply
            self._read_into_buffer()
            _sleep_ms(1)

        raise ProtocolTimeoutError("等待电机回复超时")

    def flush_input(self):
        if hasattr(self.uart, "read"):
            while True:
                chunk = self.uart.read()
                if not chunk:
                    break
        self._rx_buffer = bytearray()

    def _resolve_seq(self, seq):
        if seq is None:
            if not self.auto_seq:
                raise ValueError("auto_seq=False 时必须手动传入 seq")
            return self.next_seq()
        return int(seq) & 0xFF

    def _request(self, payload, expected_seq, expected_motor_id, expected_heads):
        self._write(payload)
        _sleep_ms(self.inter_frame_gap_ms)
        return self.read_reply(
            expected_seq=expected_seq,
            expected_motor_id=expected_motor_id,
            expected_heads=expected_heads,
        )

    def _write(self, payload):
        if hasattr(self.uart, "flush"):
            self.uart.flush()
        written = self.uart.write(payload)
        if written is not None and written != len(payload):
            raise OSError("UART 发送不完整")

    def _read_into_buffer(self):
        count = 0
        if hasattr(self.uart, "any"):
            try:
                count = int(self.uart.any())
            except TypeError:
                count = 0

        if count <= 0:
            chunk = self.uart.read()
        else:
            chunk = self.uart.read(count)

        if chunk:
            self._rx_buffer.extend(chunk)

    def _try_parse_reply(self):
        while True:
            if len(self._rx_buffer) < 2:
                return None

            header_pos = self._find_reply_header()
            if header_pos < 0:
                self._keep_possible_header_prefix()
                return None

            if header_pos > 0:
                del self._rx_buffer[:header_pos]

            if len(self._rx_buffer) < 2:
                return None

            head = (int(self._rx_buffer[0]) << 8) | int(self._rx_buffer[1])
            spec = REPLY_FRAME_SPECS.get(head)
            if spec is None:
                del self._rx_buffer[0]
                continue

            frame_size = spec[0]
            if len(self._rx_buffer) < frame_size:
                return None

            frame_bytes = bytes(self._rx_buffer[:frame_size])
            try:
                reply = decode_reply_frame(frame_bytes)
            except ValueError as exc:
                del self._rx_buffer[0]
                raise ProtocolDecodeError(str(exc))

            del self._rx_buffer[:frame_size]
            return reply

    def _find_reply_header(self):
        index = 0
        length = len(self._rx_buffer) - 1
        while index < length:
            head = (int(self._rx_buffer[index]) << 8) | int(self._rx_buffer[index + 1])
            if head in REPLY_FRAME_SPECS:
                return index
            index += 1
        return -1

    def _keep_possible_header_prefix(self):
        if self._rx_buffer and int(self._rx_buffer[-1]) in (0x06, 0x07, 0x08, 0x09, 0x0A):
            self._rx_buffer[:] = self._rx_buffer[-1:]
            return
        self._rx_buffer = bytearray()

    def _matches(self, reply, expected_seq, expected_motor_id, expected_heads):
        if expected_heads:
            if int(reply.get("head", -1)) not in expected_heads:
                return False
        if expected_seq is not None and int(reply.get("seq", -1)) != int(expected_seq):
            return False
        if expected_motor_id is not None and int(reply.get("motor_id", -1)) != int(expected_motor_id):
            return False
        return True


FRAME_SIZE_BY_HEAD = {
    CTRL_CMD_HEAD: CTRL_CMD_FRAME_SIZE,
    PARAM_WRITE_CMD_HEAD: PARAM_WRITE_CMD_FRAME_SIZE,
    QUERY_CMD_HEAD: QUERY_CMD_FRAME_SIZE,
    CALIB_CMD_HEAD: CALIB_CMD_FRAME_SIZE,
}
