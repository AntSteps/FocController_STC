#!/usr/bin/env python3
"""FOC Controller PC 端协议传输服务层。"""

from __future__ import annotations

from dataclasses import dataclass
from queue import Empty, Queue
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import serial
import serial.tools.list_ports
from protocol_codec import (
    ACK_REPLY_HEAD,
    CALIB_CMD_FRAME_SIZE,
    CALIB_CMD_HEAD,
    CALIB_CMD_START_AUTO,
    CALIB_CMD_SUMMARY_LABELS,
    CALIB_STATUS_REPLY_HEAD,
    CALIB_SAMPLE_REPLY_HEAD,
    CONTROL_MODE_SUMMARY_LABELS,
    CTRL_CMD_FRAME_SIZE,
    CTRL_CMD_HEAD,
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
    QUERY_SUMMARY_LABELS,
    REPLY_FRAME_SPECS,
    STATE_REPLY_HEAD,
    build_calib_frame,
    build_ctrl_cmd_frame,
    build_param_write_frame,
    build_query_frame,
    decode_ack_reply,
    decode_calib_status_reply,
    decode_calib_sample_reply,
    decode_param_snapshot_reply,
    decode_param_value_reply,
    decode_state_reply,
    parse_hex_string,
)


DEFAULT_POLL_HZ = 10.0
DEFAULT_CALIB_POLL_HZ = 2.0
REQUEST_TIMEOUT_SECONDS = 1.5
MIN_HALF_DUPLEX_GAP_SECONDS = 0.003


@dataclass
class CtrlCmdFrame:
    motor_id: int
    ctrl_mode: int
    target: float
    load_ff: float
    seq: int
    head: int = CTRL_CMD_HEAD

    def to_bytes(self) -> bytes:
        return build_ctrl_cmd_frame(
            self.motor_id,
            self.ctrl_mode,
            self.target,
            self.load_ff,
            self.seq,
        )

    def to_summary(self, labels: Dict[int, str]) -> str:
        mode_label = labels.get(self.ctrl_mode, f"模式 {self.ctrl_mode}")
        return (
            f"控制命令 ID={self.motor_id} seq={self.seq} "
            f"mode={mode_label} target={self.target:.3f} "
            f"load_ff={self.load_ff:.3f}"
        )


@dataclass
class ParamWriteCmdFrame:
    motor_id: int
    param_id: int
    value: float
    seq: int
    head: int = PARAM_WRITE_CMD_HEAD

    def to_bytes(self) -> bytes:
        return build_param_write_frame(self.motor_id, self.param_id, self.value, self.seq)

    def to_summary(self, labels: Dict[int, str]) -> str:
        param_label = labels.get(self.param_id, f"参数 {self.param_id}")
        return (
            f"参数写入 ID={self.motor_id} seq={self.seq} "
            f"param={param_label} value={self.value:.6f}"
        )


@dataclass
class QueryCmdFrame:
    motor_id: int
    query_type: int
    arg0: int
    seq: int
    head: int = QUERY_CMD_HEAD

    def to_bytes(self) -> bytes:
        return build_query_frame(self.motor_id, self.query_type, self.arg0, self.seq)

    def to_summary(self, query_labels: Dict[int, str], arg_label: str = "") -> str:
        query_name = query_labels.get(self.query_type, f"查询 {self.query_type}")
        suffix = f" arg={arg_label}" if arg_label else ""
        return f"查询命令 ID={self.motor_id} seq={self.seq} {query_name}{suffix}"


@dataclass
class CalibCmdFrame:
    motor_id: int
    calib_cmd: int
    arg0: int
    seq: int
    head: int = CALIB_CMD_HEAD

    def to_bytes(self) -> bytes:
        return build_calib_frame(self.motor_id, self.calib_cmd, self.arg0, self.seq)

    def to_summary(self, labels: Dict[int, str]) -> str:
        calib_name = labels.get(self.calib_cmd, f"校准命令 {self.calib_cmd}")
        return f"校准命令 ID={self.motor_id} seq={self.seq} {calib_name}"


@dataclass
class AckReply:
    motor_id: int
    cmd_type: int
    result_code: int
    seq: int
    crc8: int
    head: int = ACK_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "AckReply":
        values = decode_ack_reply(data)
        return cls(
            motor_id=int(values["motor_id"]),
            cmd_type=int(values["cmd_type"]),
            result_code=int(values["result_code"]),
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


@dataclass
class StateReply:
    motor_id: int
    control_mode: int
    target: float
    velocity: float
    output: float
    position: float
    voltage: float
    health_status: int
    seq: int
    crc8: int
    head: int = STATE_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "StateReply":
        values = decode_state_reply(data)
        return cls(
            motor_id=int(values["motor_id"]),
            control_mode=int(values["control_mode"]),
            target=float(values["target"]),
            velocity=float(values["velocity"]),
            output=float(values["output"]),
            position=float(values["position"]),
            voltage=float(values["voltage"]),
            health_status=int(values["health_status"]),
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


@dataclass
class ParamValueReply:
    motor_id: int
    param_id: int
    value: float
    seq: int
    crc8: int
    head: int = PARAM_VALUE_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "ParamValueReply":
        values = decode_param_value_reply(data)
        return cls(
            motor_id=int(values["motor_id"]),
            param_id=int(values["param_id"]),
            value=float(values["value"]),
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


@dataclass
class ParamSnapshot:
    motor_pole_pairs: int
    motor_phase_resistance: float
    motor_phase_inductance: float
    speed_kp: float
    speed_ki: float
    speed_out_max: float
    speed_out_min: float
    speed_lpf_time_constant_ms: float
    ff_acceleration_gain: float
    ff_friction_compensation: float
    speed_beta: float
    speed_cmd_lpf_ms: float
    pll_bandwidth_hz: float
    pll_zeta: float
    speed_R: float
    speed_Ke: float
    velocity_limit: float
    encoder_offset: int
    calib_pole_pairs: int
    calib_dir: int
    calib_electrical_offset_rad: float
    calib_harmonic_amp_1: float
    calib_harmonic_phase_1: float
    calib_harmonic_amp_2: float
    calib_harmonic_phase_2: float

    def to_lines(self) -> List[str]:
        return [
            f"motor_pole_pairs = {self.motor_pole_pairs}",
            f"motor_phase_resistance = {self.motor_phase_resistance:.6f}",
            f"motor_phase_inductance = {self.motor_phase_inductance:.6f}",
            f"speed_kp = {self.speed_kp:.6f}",
            f"speed_ki = {self.speed_ki:.6f}",
            f"speed_out_max = {self.speed_out_max:.6f}",
            f"speed_out_min = {self.speed_out_min:.6f}",
            f"speed_lpf_time_constant_ms = {self.speed_lpf_time_constant_ms:.6f}",
            f"ff_acceleration_gain = {self.ff_acceleration_gain:.6f}",
            f"ff_friction_compensation = {self.ff_friction_compensation:.6f}",
            f"speed_beta = {self.speed_beta:.6f}",
            f"speed_cmd_lpf_ms = {self.speed_cmd_lpf_ms:.6f}",
            f"pll_bandwidth_hz = {self.pll_bandwidth_hz:.6f}",
            f"pll_zeta = {self.pll_zeta:.6f}",
            f"speed_R = {self.speed_R:.6f}",
            f"speed_Ke = {self.speed_Ke:.6f}",
            f"velocity_limit = {self.velocity_limit:.6f}",
            f"encoder_offset = {self.encoder_offset}",
            f"calib_pole_pairs = {self.calib_pole_pairs}",
            f"calib_dir = {self.calib_dir}",
            f"calib_electrical_offset_rad = {self.calib_electrical_offset_rad:.6f}",
            f"calib_harmonic_amp_1 = {self.calib_harmonic_amp_1:.6f}",
            f"calib_harmonic_phase_1 = {self.calib_harmonic_phase_1:.6f}",
            f"calib_harmonic_amp_2 = {self.calib_harmonic_amp_2:.6f}",
            f"calib_harmonic_phase_2 = {self.calib_harmonic_phase_2:.6f}",
        ]


@dataclass
class ParamSnapshotReply:
    motor_id: int
    snapshot_version: int
    snapshot: ParamSnapshot
    seq: int
    crc8: int
    head: int = PARAM_SNAPSHOT_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "ParamSnapshotReply":
        values = decode_param_snapshot_reply(data)
        snapshot_values = values["snapshot"]
        snapshot = ParamSnapshot(
            motor_pole_pairs=int(snapshot_values["motor_pole_pairs"]),
            motor_phase_resistance=float(snapshot_values["motor_phase_resistance"]),
            motor_phase_inductance=float(snapshot_values["motor_phase_inductance"]),
            speed_kp=float(snapshot_values["speed_kp"]),
            speed_ki=float(snapshot_values["speed_ki"]),
            speed_out_max=float(snapshot_values["speed_out_max"]),
            speed_out_min=float(snapshot_values["speed_out_min"]),
            speed_lpf_time_constant_ms=float(snapshot_values["speed_lpf_time_constant_ms"]),
            ff_acceleration_gain=float(snapshot_values["ff_acceleration_gain"]),
            ff_friction_compensation=float(snapshot_values["ff_friction_compensation"]),
            speed_beta=float(snapshot_values["speed_beta"]),
            speed_cmd_lpf_ms=float(snapshot_values["speed_cmd_lpf_ms"]),
            pll_bandwidth_hz=float(snapshot_values["pll_bandwidth_hz"]),
            pll_zeta=float(snapshot_values["pll_zeta"]),
            speed_R=float(snapshot_values["speed_R"]),
            speed_Ke=float(snapshot_values["speed_Ke"]),
            velocity_limit=float(snapshot_values["velocity_limit"]),
            encoder_offset=int(snapshot_values["encoder_offset"]),
            calib_pole_pairs=int(snapshot_values["calib_pole_pairs"]),
            calib_dir=int(snapshot_values["calib_dir"]),
            calib_electrical_offset_rad=float(snapshot_values["calib_electrical_offset_rad"]),
            calib_harmonic_amp_1=float(snapshot_values["calib_harmonic_amp_1"]),
            calib_harmonic_phase_1=float(snapshot_values["calib_harmonic_phase_1"]),
            calib_harmonic_amp_2=float(snapshot_values["calib_harmonic_amp_2"]),
            calib_harmonic_phase_2=float(snapshot_values["calib_harmonic_phase_2"]),
        )
        return cls(
            motor_id=int(values["motor_id"]),
            snapshot_version=int(values["snapshot_version"]),
            snapshot=snapshot,
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


@dataclass
class CalibStatusReply:
    motor_id: int
    calib_state: int
    progress: int
    seq: int
    crc8: int
    head: int = CALIB_STATUS_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "CalibStatusReply":
        values = decode_calib_status_reply(data)
        return cls(
            motor_id=int(values["motor_id"]),
            calib_state=int(values["calib_state"]),
            progress=int(values["progress"]),
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


@dataclass
class CalibSampleReply:
    motor_id: int
    sample_index: int
    direction: int
    command_theta: float
    mechanical_angle: float
    electrical_error: float
    seq: int
    crc8: int
    head: int = CALIB_SAMPLE_REPLY_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "CalibSampleReply":
        values = decode_calib_sample_reply(data)
        return cls(
            motor_id=int(values["motor_id"]),
            sample_index=int(values["sample_index"]),
            direction=int(values["direction"]),
            command_theta=float(values["command_theta"]),
            mechanical_angle=float(values["mechanical_angle"]),
            electrical_error=float(values["electrical_error"]),
            seq=int(values["seq"]),
            crc8=int(values["crc8"]),
        )


ParsedReply = Union[AckReply, StateReply, ParamValueReply, ParamSnapshotReply, CalibStatusReply, CalibSampleReply]


@dataclass
class PendingRequest:
    seq: int
    summary: str
    timestamp: float


class SerialWorker:
    def __init__(
        self,
        port: str,
        baudrate: int,
        on_event,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.on_event = on_event
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._tx_queue: "Queue[Tuple[bytes, str, Optional[int]]]" = Queue()
        self._stop_event = threading.Event()
        self._buffer = bytearray()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="serial-worker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

    def send(self, payload: bytes, summary: str, expected_seq: Optional[int]) -> None:
        self._tx_queue.put((payload, summary, expected_seq))

    def _run(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.005,
                write_timeout=0.2,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as exc:
            self.on_event("connection", {"connected": False, "message": f"串口打开失败: {exc}"})
            return

        self.on_event(
            "connection",
            {"connected": True, "message": f"已连接 {self.port} @ {self.baudrate}bps"},
        )

        try:
            while not self._stop_event.is_set():
                self._drain_tx_queue()
                incoming = self._serial.read(max(1, int(self._serial.in_waiting or 0)))
                if incoming:
                    self._buffer.extend(incoming)
                    self._process_rx_buffer()
        except serial.SerialException as exc:
            self.on_event("error", f"串口通信异常: {exc}")
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass
                self._serial = None
            self.on_event("connection", {"connected": False, "message": "串口已断开"})

    def _drain_tx_queue(self) -> None:
        while True:
            try:
                payload, summary, expected_seq = self._tx_queue.get_nowait()
            except Empty:
                break

            if self._serial is None:
                self.on_event(
                    "tx_error",
                    {"expected_seq": expected_seq, "message": "串口未连接，发送失败"},
                )
                continue

            try:
                self._serial.write(payload)
                self._serial.flush()
            except serial.SerialTimeoutException as exc:
                self.on_event(
                    "tx_error",
                    {"expected_seq": expected_seq, "message": f"串口发送超时: {exc}"},
                )
                continue
            except serial.SerialException as exc:
                self.on_event(
                    "tx_error",
                    {"expected_seq": expected_seq, "message": f"串口发送失败: {exc}"},
                )
                continue

            self.on_event(
                "tx",
                {
                    "summary": summary,
                    "payload": payload,
                    "expected_seq": expected_seq,
                },
            )

    def _find_reply_header(self) -> int:
        for index in range(len(self._buffer) - 1):
            head = (int(self._buffer[index]) << 8) | int(self._buffer[index + 1])
            if head in REPLY_FRAME_SPECS:
                return index
        return -1

    def _keep_possible_header_prefix(self) -> int:
        if self._buffer and int(self._buffer[-1]) in (0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B):
            last_byte = self._buffer[-1]
            self._buffer.clear()
            self._buffer.append(last_byte)
            return 1
        self._buffer.clear()
        return 0

    def _decode_reply(self, frame_bytes: bytes) -> ParsedReply:
        if len(frame_bytes) < 2:
            raise ValueError("回复帧长度不足")

        head = (frame_bytes[0] << 8) | frame_bytes[1]
        if head == ACK_REPLY_HEAD:
            return AckReply.from_bytes(frame_bytes)
        if head == STATE_REPLY_HEAD:
            return StateReply.from_bytes(frame_bytes)
        if head == PARAM_VALUE_REPLY_HEAD:
            return ParamValueReply.from_bytes(frame_bytes)
        if head == PARAM_SNAPSHOT_REPLY_HEAD:
            return ParamSnapshotReply.from_bytes(frame_bytes)
        if head == CALIB_STATUS_REPLY_HEAD:
            return CalibStatusReply.from_bytes(frame_bytes)
        if head == CALIB_SAMPLE_REPLY_HEAD:
            return CalibSampleReply.from_bytes(frame_bytes)
        raise ValueError(f"未知回复帧头: 0x{head:04X}")

    def _process_rx_buffer(self) -> None:
        while True:
            if len(self._buffer) < 2:
                break

            header_pos = self._find_reply_header()
            if header_pos < 0:
                dropped = len(self._buffer)
                kept = self._keep_possible_header_prefix()
                if dropped > kept:
                    self.on_event("parse_error", f"未找到上行帧头，丢弃 {dropped - kept} 字节")
                break

            if header_pos > 0:
                del self._buffer[:header_pos]
                self.on_event("parse_error", f"跳过 {header_pos} 字节噪声数据")

            if len(self._buffer) < 2:
                break

            head = (int(self._buffer[0]) << 8) | int(self._buffer[1])
            spec = REPLY_FRAME_SPECS.get(head)
            if spec is None:
                del self._buffer[0]
                self.on_event("parse_error", f"未知回复帧头: 0x{head:04X}")
                continue

            frame_size, _ = spec
            if len(self._buffer) < frame_size:
                break

            frame_bytes = bytes(self._buffer[:frame_size])
            try:
                reply = self._decode_reply(frame_bytes)
            except ValueError as exc:
                del self._buffer[0]
                self.on_event("parse_error", str(exc))
                continue

            del self._buffer[:frame_size]
            self.on_event("rx", {"reply": reply, "payload": frame_bytes})


class ProtocolService:
    def __init__(self, request_timeout: float = REQUEST_TIMEOUT_SECONDS) -> None:
        self.request_timeout = request_timeout
        self.worker: Optional[SerialWorker] = None
        self.connected = False
        self.sequence_counter = 0
        self.pending_request: Optional[PendingRequest] = None
        self.ready_after = 0.0
        self._worker_events: "Queue[Tuple[str, object]]" = Queue()
        self._events: "Queue[Tuple[str, object]]" = Queue()

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def is_waiting_reply(self) -> bool:
        return (self.pending_request is not None) or (time.monotonic() < self.ready_after)

    def connect(self, port: str, baudrate: int) -> None:
        if self.worker is not None:
            self._emit("error", "已有连接实例在运行")
            return

        self.connected = False
        self.pending_request = None
        self.ready_after = 0.0
        self.worker = SerialWorker(port=port, baudrate=baudrate, on_event=self._post_worker_event)
        self.worker.start()

    def disconnect(self) -> None:
        worker = self.worker
        self.worker = None
        self.connected = False
        self.pending_request = None
        self.ready_after = 0.0
        if worker is not None:
            worker.stop()

    def allocate_seq(self, auto_enabled: bool, manual_seq: int) -> int:
        if auto_enabled:
            seq = self.sequence_counter & 0xFF
            self.sequence_counter = (self.sequence_counter + 1) & 0xFF
            return seq
        return manual_seq & 0xFF

    def send_control(
        self,
        motor_id: int,
        ctrl_mode: int,
        target: float,
        load_ff: float,
        seq: int,
    ) -> None:
        frame = CtrlCmdFrame(
            motor_id=motor_id,
            ctrl_mode=ctrl_mode,
            target=target,
            load_ff=load_ff,
            seq=seq,
        )
        self._queue_frame(frame.to_bytes(), frame.to_summary(CONTROL_MODE_SUMMARY_LABELS), seq)

    def send_param_write(
        self,
        motor_id: int,
        param_id: int,
        value: float,
        seq: int,
    ) -> None:
        frame = ParamWriteCmdFrame(
            motor_id=motor_id,
            param_id=param_id,
            value=value,
            seq=seq,
        )
        summary = (
            f"参数写入 ID={motor_id} seq={seq} "
            f"param=0x{param_id:02X} value={value:.6f}"
        )
        self._queue_frame(frame.to_bytes(), summary, seq)

    def query_state(self, motor_id: int, seq: int) -> None:
        self._queue_query(motor_id, QUERY_GET_STATE, 0, seq)

    def query_param(
        self,
        motor_id: int,
        param_id: int,
        seq: int,
    ) -> None:
        self._queue_query(
            motor_id,
            QUERY_GET_PARAM,
            param_id,
            seq,
            f"arg=0x{param_id:02X}",
        )

    def query_snapshot(self, motor_id: int, seq: int) -> None:
        self._queue_query(motor_id, QUERY_GET_PARAM_SNAPSHOT, 0, seq)

    def query_calibration_status(
        self,
        motor_id: int,
        seq: int,
    ) -> None:
        self._queue_query(motor_id, QUERY_GET_CALIB_STATUS, 0, seq)

    def start_calibration(
        self,
        motor_id: int,
        seq: int,
    ) -> None:
        frame = CalibCmdFrame(
            motor_id=motor_id,
            calib_cmd=CALIB_CMD_START_AUTO,
            arg0=0,
            seq=seq,
        )
        self._queue_frame(frame.to_bytes(), frame.to_summary(CALIB_CMD_SUMMARY_LABELS), seq)

    def send_raw_hex(self, text: str) -> None:
        try:
            payload = parse_hex_string(text)
        except ValueError as exc:
            self._emit("error", str(exc))
            return
        self._queue_frame(payload, "原始帧发送", None)

    def poll_events(self) -> List[Tuple[str, object]]:
        events: List[Tuple[str, object]] = []
        self._process_worker_events()
        self._release_timeouts()

        while True:
            try:
                events.append(self._events.get_nowait())
            except Empty:
                break
        return events

    def _queue_query(
        self,
        motor_id: int,
        query_type: int,
        arg0: int,
        seq: int,
        detail: str = "",
    ) -> None:
        frame = QueryCmdFrame(
            motor_id=motor_id,
            query_type=query_type,
            arg0=arg0,
            seq=seq,
        )
        self._queue_frame(frame.to_bytes(), frame.to_summary(QUERY_SUMMARY_LABELS, detail), seq)

    def _queue_frame(self, payload: bytes, summary: str, expected_seq: Optional[int]) -> None:
        if self.worker is None:
            self._emit("error", "串口未连接，无法发送")
            return

        if time.monotonic() < self.ready_after:
            self._emit("error", "半双工切换等待中，请稍后再发送")
            return

        if self.pending_request is not None:
            age_ms = (time.monotonic() - self.pending_request.timestamp) * 1000.0
            self._emit(
                "error",
                (
                    "半双工等待响应中，暂不发送新请求: "
                    f"seq={self.pending_request.seq} {self.pending_request.summary} ({age_ms:.1f} ms)"
                ),
            )
            return

        if expected_seq is None:
            expected_seq = self._infer_expected_seq(payload)
            if expected_seq is None:
                self._emit("error", "原始帧无法识别 seq，半双工模式下拒绝发送")
                return

        self.pending_request = PendingRequest(
            seq=expected_seq,
            summary=summary,
            timestamp=time.monotonic(),
        )
        self.worker.send(payload, summary, expected_seq)

    def _infer_expected_seq(self, payload: bytes) -> Optional[int]:
        if len(payload) < 2:
            return None

        head = (int(payload[0]) << 8) | int(payload[1])
        expected_lengths = {
            CTRL_CMD_HEAD: CTRL_CMD_FRAME_SIZE,
            PARAM_WRITE_CMD_HEAD: PARAM_WRITE_CMD_FRAME_SIZE,
            QUERY_CMD_HEAD: QUERY_CMD_FRAME_SIZE,
            CALIB_CMD_HEAD: CALIB_CMD_FRAME_SIZE,
        }
        expected_length = expected_lengths.get(head)
        if expected_length is None or len(payload) != expected_length:
            return None
        return int(payload[-2])

    def _release_timeouts(self) -> None:
        if self.pending_request is None:
            return

        elapsed = time.monotonic() - self.pending_request.timestamp
        if elapsed < self.request_timeout:
            return

        timed_out = self.pending_request
        self.pending_request = None
        self.ready_after = time.monotonic() + MIN_HALF_DUPLEX_GAP_SECONDS
        self._emit(
            "timeout",
            {
                "seq": timed_out.seq,
                "summary": timed_out.summary,
                "elapsed_ms": elapsed * 1000.0,
            },
        )

    def _process_worker_events(self) -> None:
        while True:
            try:
                event_type, payload = self._worker_events.get_nowait()
            except Empty:
                break

            if event_type == "connection":
                self._handle_connection_event(payload)
            elif event_type == "tx_error":
                self._handle_tx_error_event(payload)
            elif event_type == "rx":
                self._handle_rx_event(payload)
            else:
                self._emit(event_type, payload)

    def _handle_connection_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        self.connected = bool(info.get("connected"))
        if not self.connected:
            self.pending_request = None
            self.ready_after = 0.0
            self.worker = None
        self._emit("connection", info)

    def _handle_tx_error_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        expected_seq = info.get("expected_seq")
        if (
            expected_seq is not None
            and self.pending_request is not None
            and int(expected_seq) == self.pending_request.seq
        ):
            self.pending_request = None
            self.ready_after = time.monotonic() + MIN_HALF_DUPLEX_GAP_SECONDS
        self._emit("error", str(info.get("message", "串口发送失败")))

    def _handle_rx_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        reply = info.get("reply")

        matched_summary = None
        latency_ms = None
        if reply is not None and self.pending_request is not None:
            reply_seq = getattr(reply, "seq", None)
            if reply_seq == self.pending_request.seq:
                matched_summary = self.pending_request.summary
                latency_ms = (time.monotonic() - self.pending_request.timestamp) * 1000.0
                self.pending_request = None
                self.ready_after = time.monotonic() + MIN_HALF_DUPLEX_GAP_SECONDS

        enriched = dict(info)
        enriched["matched_summary"] = matched_summary
        enriched["latency_ms"] = latency_ms
        self._emit("rx", enriched)

    def _post_worker_event(self, event_type: str, payload: object) -> None:
        self._worker_events.put((event_type, payload))

    def _emit(self, event_type: str, payload: object) -> None:
        self._events.put((event_type, payload))


def list_serial_ports() -> List[str]:
    return [port.device for port in serial.tools.list_ports.comports()]
