#!/usr/bin/env python3
"""FOC Controller DearPyGui 上位机。"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from queue import Empty, Queue
from typing import Callable, Deque, Dict, List, Optional

import serial
import serial.tools.list_ports

try:
    from dearpygui import dearpygui as dpg
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "缺少 DearPyGui，请先执行: python -m pip install -r requirements.txt"
    ) from exc


REPLY_FRAME_HEAD = 0x0721
CTRL_CMD_FRAME_HEAD = 0x1145
PARAM_CMD_FRAME_HEAD = 0x1245

REPLY_FRAME_FORMAT = ">H B f f f f B B B"
CTRL_CMD_FRAME_FORMAT = ">H B B f f f B B"
PARAM_CMD_FRAME_FORMAT = ">H B B f B B"

REPLY_FRAME_SIZE = struct.calcsize(REPLY_FRAME_FORMAT)
CTRL_CMD_FRAME_SIZE = struct.calcsize(CTRL_CMD_FRAME_FORMAT)
PARAM_CMD_FRAME_SIZE = struct.calcsize(PARAM_CMD_FRAME_FORMAT)

MAX_LOG_LINES = 250
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_HISTORY_LENGTH = 500

STATUS_FLAGS = {
    0x01: "编码器异常",
    0x02: "电机异常",
}

WINDOWS_FONT_CANDIDATES = [
    Path("C:/Windows/Fonts/msyh.ttc"),
    Path("C:/Windows/Fonts/msyhbd.ttc"),
    Path("C:/Windows/Fonts/msyh.ttf"),
    Path("C:/Windows/Fonts/simhei.ttf"),
    Path("C:/Windows/Fonts/simsun.ttc"),
    Path("C:/Windows/Fonts/Deng.ttf"),
    Path("C:/Windows/Fonts/Dengb.ttf"),
]


def crc8_check(data: bytes, has_crc: int = 0) -> int:
    crc = 0x00
    length = len(data) - has_crc
    for index in range(length):
        crc ^= data[index]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def parse_status(status: int) -> str:
    if status == 0:
        return "就绪"
    active_flags = [name for bit, name in STATUS_FLAGS.items() if status & bit]
    return ", ".join(active_flags) if active_flags else "未知状态"


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def parse_hex_string(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = [token for token in cleaned.split() if token]
    if not tokens:
        raise ValueError("请输入十六进制字节，例如 11 45 01 00 ...")
    return bytes(int(token, 16) & 0xFF for token in tokens)


def find_cjk_font() -> Optional[Path]:
    for font_path in WINDOWS_FONT_CANDIDATES:
        if font_path.exists():
            return font_path
    return None


@dataclass(frozen=True)
class ChoiceOption:
    value: int
    label: str
    description: str = ""


@dataclass
class ReplyFrame:
    motor_id: int
    velocity: float
    output: float
    position: float
    voltage: float
    status: int
    seq: int
    crc8: int
    head: int = REPLY_FRAME_HEAD

    @classmethod
    def from_bytes(cls, data: bytes) -> "ReplyFrame":
        if len(data) != REPLY_FRAME_SIZE:
            raise ValueError(f"ReplyFrame 长度错误: {len(data)}")
        head, motor_id, velocity, output, position, voltage, status, seq, crc8 = struct.unpack(
            REPLY_FRAME_FORMAT, data
        )
        if head != REPLY_FRAME_HEAD:
            raise ValueError(f"ReplyFrame 帧头错误: 0x{head:04X}")
        calc_crc = crc8_check(data, has_crc=1)
        if calc_crc != crc8:
            raise ValueError(f"ReplyFrame CRC 错误: 计算 0x{calc_crc:02X}, 接收 0x{crc8:02X}")
        return cls(
            motor_id=motor_id,
            velocity=velocity,
            output=output,
            position=position,
            voltage=voltage,
            status=status,
            seq=seq,
            crc8=crc8,
        )

    def to_summary(self) -> str:
        return (
            f"ID={self.motor_id} seq={self.seq} vel={self.velocity:.3f} "
            f"out={self.output:.3f} pos={self.position:.3f} Vbus={self.voltage:.3f}"
        )


@dataclass
class CtrlCmdFrame:
    motor_id: int
    ctrl_mode: int
    target: float
    ff0: float
    ff1: float
    seq: int
    head: int = CTRL_CMD_FRAME_HEAD

    def to_bytes(self) -> bytes:
        payload = struct.pack(
            CTRL_CMD_FRAME_FORMAT,
            self.head,
            self.motor_id & 0xFF,
            self.ctrl_mode & 0xFF,
            float(self.target),
            float(self.ff0),
            float(self.ff1),
            self.seq & 0xFF,
            0,
        )
        crc = crc8_check(payload, has_crc=1)
        return payload[:-1] + bytes([crc])

    def to_summary(self, labels: Dict[int, str]) -> str:
        mode_label = labels.get(self.ctrl_mode, f"模式 {self.ctrl_mode}")
        return (
            f"控制命令 ID={self.motor_id} seq={self.seq} mode={mode_label} "
            f"target={self.target:.3f} ff=[{self.ff0:.3f}, {self.ff1:.3f}]"
        )


@dataclass
class ParamCmdFrame:
    motor_id: int
    param_id: int
    value: float
    seq: int
    head: int = PARAM_CMD_FRAME_HEAD

    def to_bytes(self) -> bytes:
        payload = struct.pack(
            PARAM_CMD_FRAME_FORMAT,
            self.head,
            self.motor_id & 0xFF,
            self.param_id & 0xFF,
            float(self.value),
            self.seq & 0xFF,
            0,
        )
        crc = crc8_check(payload, has_crc=1)
        return payload[:-1] + bytes([crc])

    def to_summary(self, labels: Dict[int, str]) -> str:
        param_label = labels.get(self.param_id, f"参数 {self.param_id}")
        return (
            f"参数命令 ID={self.motor_id} seq={self.seq} param={param_label} "
            f"value={self.value:.6f}"
        )


@dataclass
class PendingRequest:
    seq: int
    summary: str
    timestamp: float


@dataclass
class TelemetryState:
    frame_count: int = 0
    error_count: int = 0
    lost_frames: int = 0
    last_seq: Optional[int] = None
    last_frame: Optional[ReplyFrame] = None
    last_ack: str = "等待首个回显"
    histories: Dict[str, Deque[float]] = field(
        default_factory=lambda: {
            "velocity": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "output": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "position": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "voltage": deque(maxlen=DEFAULT_HISTORY_LENGTH),
        }
    )


@dataclass
class AppConfig:
    control_modes: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(0, "停止/休眠", "ctrl=0"),
            ChoiceOption(1, "速度环", "ctrl=1"),
        ]
    )
    param_defs: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(0x01, "速度环 Kp", "PARAM_CMD_ID_VEL_KP"),
            ChoiceOption(0x02, "速度环 Ki", "PARAM_CMD_ID_VEL_KI"),
            ChoiceOption(0x03, "速度限制", "PARAM_CMD_ID_VEL_LIMIT"),
            ChoiceOption(0x04, "电流限制", "PARAM_CMD_ID_CUR_LIMIT"),
            ChoiceOption(0x05, "前馈增益", "PARAM_CMD_ID_FF_GAIN"),
            ChoiceOption(0x06, "前馈使能", "PARAM_CMD_ID_FF_ENABLE"),
        ]
    )

    @property
    def control_mode_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.control_modes}

    @property
    def param_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.param_defs}


class SerialWorker:
    def __init__(
        self,
        port: str,
        baudrate: int,
        on_event: Callable[[str, object], None],
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.on_event = on_event
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._tx_queue: "Queue[tuple[bytes, str, Optional[int]]]" = Queue()
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
                timeout=0.05,
                write_timeout=0.2,
            )
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
                incoming = self._serial.read(256)
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
                self.on_event("error", "串口未连接，发送失败")
                continue
            try:
                self._serial.write(payload)
                self._serial.flush()
            except serial.SerialTimeoutException as exc:
                self.on_event("error", f"串口发送超时: {exc}")
                continue
            except serial.SerialException as exc:
                self.on_event("error", f"串口发送失败: {exc}")
                continue
            self.on_event(
                "tx",
                {
                    "summary": summary,
                    "payload": payload,
                    "expected_seq": expected_seq,
                },
            )

    def _process_rx_buffer(self) -> None:
        while len(self._buffer) >= REPLY_FRAME_SIZE:
            header_pos = self._find_reply_header()
            if header_pos < 0:
                dropped = len(self._buffer)
                self._buffer.clear()
                self.on_event("parse_error", f"未找到上行帧头，丢弃 {dropped} 字节")
                break
            if header_pos > 0:
                del self._buffer[:header_pos]
                self.on_event("parse_error", f"跳过 {header_pos} 字节噪声数据")
            if len(self._buffer) < REPLY_FRAME_SIZE:
                break

            frame_bytes = bytes(self._buffer[:REPLY_FRAME_SIZE])
            try:
                frame = ReplyFrame.from_bytes(frame_bytes)
            except ValueError as exc:
                del self._buffer[0]
                self.on_event("parse_error", str(exc))
                continue

            del self._buffer[:REPLY_FRAME_SIZE]
            self.on_event("rx", {"frame": frame, "payload": frame_bytes})

    def _find_reply_header(self) -> int:
        for index in range(len(self._buffer) - 1):
            if self._buffer[index] == 0x07 and self._buffer[index + 1] == 0x21:
                return index
        return -1


class FOCGuiApp:
    def __init__(self, initial_port: str = "", initial_baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.config = AppConfig()
        self.state = TelemetryState()
        self.initial_port = initial_port
        self.initial_baudrate = initial_baudrate
        self.worker: Optional[SerialWorker] = None
        self.events: "Queue[tuple[str, object]]" = Queue()
        self.log_lines: Deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.pending_requests: Dict[int, PendingRequest] = {}
        self.sequence_counter = 0
        self.plot_dirty = False
        self.font_status = "未加载中文字体"

        self.tags = {
            "port_combo": "port_combo",
            "baud_input": "baud_input",
            "connect_button": "connect_button",
            "status_text": "status_text",
            "motor_id": "motor_id",
            "auto_seq": "auto_seq",
            "seq_input": "seq_input",
            "control_mode": "control_mode",
            "control_target": "control_target",
            "control_ff0": "control_ff0",
            "control_ff1": "control_ff1",
            "param_choice": "param_choice",
            "param_value": "param_value",
            "raw_hex": "raw_hex",
            "frame_count": "frame_count",
            "error_count": "error_count",
            "lost_frames": "lost_frames",
            "last_ack": "last_ack",
            "value_motor_id": "value_motor_id",
            "value_seq": "value_seq",
            "value_status": "value_status",
            "value_velocity": "value_velocity",
            "value_output": "value_output",
            "value_position": "value_position",
            "value_voltage": "value_voltage",
            "value_hex": "value_hex",
            "log_text": "log_text",
            "velocity_series": "velocity_series",
            "output_series": "output_series",
            "position_series": "position_series",
            "voltage_series": "voltage_series",
        }

    def post_event(self, event_type: str, payload: object) -> None:
        self.events.put((event_type, payload))

    def run(self) -> None:
        dpg.create_context()
        self._setup_fonts()
        self._build_theme()
        self._build_ui()
        dpg.set_primary_window("main_window", True)
        self.refresh_ports(select_port=self.initial_port)
        dpg.set_value(self.tags["baud_input"], int(self.initial_baudrate))
        dpg.create_viewport(title="FOC Controller Host", width=1480, height=920, min_width=1200, min_height=760)
        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():
            self._process_events()
            if self.plot_dirty:
                self._update_plots()
                self.plot_dirty = False
            dpg.render_dearpygui_frame()

        self.disconnect()
        dpg.destroy_context()

    def _setup_fonts(self) -> None:
        font_path = find_cjk_font()
        if font_path is None:
            self.font_status = "未找到系统中文字体，界面可能出现乱码"
            return

        with dpg.font_registry():
            with dpg.font(str(font_path), 18) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                dpg.add_font_range(0x0020, 0x00FF)
                dpg.add_font_range(0x2000, 0x206F)
                dpg.add_font_range(0x3000, 0x30FF)
                dpg.add_font_range(0x4E00, 0x9FFF)
                dpg.add_font_range(0xFF00, 0xFFEF)
        dpg.bind_font(default_font)
        self.font_status = f"已加载中文字体: {font_path.name}"

    def _build_theme(self) -> None:
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 12, 12)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 6)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 10, 10)
                dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 8, 6)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 10)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 10)
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (20, 25, 32, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (28, 35, 44, 255))
                dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (28, 35, 44, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (36, 46, 58, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (48, 61, 77, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (54, 87, 120, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (74, 114, 155, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (43, 67, 92, 255))
                dpg.add_theme_color(dpg.mvThemeCol_PlotLines, (228, 170, 71, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (232, 238, 244, 255))
                dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (137, 149, 160, 255))
        dpg.bind_theme(global_theme)

    def _build_ui(self) -> None:
        with dpg.window(
            label="FOC Controller Host",
            tag="main_window",
            no_title_bar=True,
            no_move=True,
            no_resize=True,
            no_collapse=True,
            no_close=True,
        ):
            with dpg.group(horizontal=True):
                with dpg.child_window(width=430, autosize_y=True, border=False):
                    self._build_connection_panel()
                    dpg.add_spacer(height=6)
                    self._build_command_panel()
                    dpg.add_spacer(height=6)
                    self._build_last_frame_panel()
                with dpg.child_window(width=-1, autosize_y=True, border=False):
                    self._build_plot_panel()
                    dpg.add_spacer(height=6)
                    self._build_log_panel()

    def _build_connection_panel(self) -> None:
        with dpg.collapsing_header(label="串口连接", default_open=True):
            dpg.add_text("RS485 会话")
            with dpg.group(horizontal=True):
                dpg.add_combo(items=[], width=180, tag=self.tags["port_combo"], no_arrow_button=False)
                dpg.add_button(label="刷新串口", callback=lambda: self.refresh_ports())
            dpg.add_input_int(
                label="波特率",
                tag=self.tags["baud_input"],
                default_value=self.initial_baudrate,
                step=100000,
                min_value=9600,
                min_clamped=True,
                width=180,
            )
            with dpg.group(horizontal=True):
                dpg.add_button(label="连接", tag=self.tags["connect_button"], callback=lambda: self.connect())
                dpg.add_button(label="断开", callback=lambda: self.disconnect())
            dpg.add_text("未连接", tag=self.tags["status_text"])

    def _build_command_panel(self) -> None:
        with dpg.collapsing_header(label="命令面板", default_open=True):
            dpg.add_input_int(label="电机 ID", tag=self.tags["motor_id"], default_value=1, min_value=0, max_value=255, min_clamped=True, max_clamped=True, width=180)
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="自动序列号", tag=self.tags["auto_seq"], default_value=True)
                dpg.add_input_int(label="SEQ", tag=self.tags["seq_input"], default_value=0, min_value=0, max_value=255, min_clamped=True, max_clamped=True, width=120)

            with dpg.tab_bar():
                with dpg.tab(label="控制命令"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.control_modes],
                        default_value=self.config.control_modes[0].label,
                        label="控制模式",
                        tag=self.tags["control_mode"],
                        width=220,
                    )
                    dpg.add_input_float(label="目标值", tag=self.tags["control_target"], default_value=0.0, width=220, format="%.6f")
                    dpg.add_input_float(label="前馈 0", tag=self.tags["control_ff0"], default_value=0.0, width=220, format="%.6f")
                    dpg.add_input_float(label="前馈 1", tag=self.tags["control_ff1"], default_value=0.0, width=220, format="%.6f")
                    dpg.add_button(label="发送控制命令", callback=lambda: self.send_control_command())

                with dpg.tab(label="参数命令"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.param_defs],
                        default_value=self.config.param_defs[0].label,
                        label="参数项",
                        tag=self.tags["param_choice"],
                        width=220,
                    )
                    dpg.add_input_float(label="参数值", tag=self.tags["param_value"], default_value=0.0, width=220, format="%.6f")
                    dpg.add_button(label="发送参数命令", callback=lambda: self.send_param_command())

                with dpg.tab(label="原始帧"):
                    dpg.add_text("用于协议联调或临时验证未建模的下行帧")
                    dpg.add_input_text(
                        label="HEX",
                        tag=self.tags["raw_hex"],
                        multiline=True,
                        width=-1,
                        height=140,
                        hint="例如: 11 45 01 01 00 00 20 41 00 00 00 00 00 00 00 00 23 5A",
                    )
                    dpg.add_button(label="发送原始帧", callback=lambda: self.send_raw_command())

    def _build_last_frame_panel(self) -> None:
        with dpg.collapsing_header(label="最新回显", default_open=True):
            with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp):
                dpg.add_table_column()
                dpg.add_table_column()
                self._add_kv_row("帧计数", self.tags["frame_count"], "0")
                self._add_kv_row("错误计数", self.tags["error_count"], "0")
                self._add_kv_row("丢帧数", self.tags["lost_frames"], "0")
                self._add_kv_row("最近匹配", self.tags["last_ack"], "等待首个回显")
                self._add_kv_row("电机 ID", self.tags["value_motor_id"], "-")
                self._add_kv_row("序列号", self.tags["value_seq"], "-")
                self._add_kv_row("状态", self.tags["value_status"], "-")
                self._add_kv_row("速度", self.tags["value_velocity"], "-")
                self._add_kv_row("输出", self.tags["value_output"], "-")
                self._add_kv_row("位置", self.tags["value_position"], "-")
                self._add_kv_row("电压", self.tags["value_voltage"], "-")
                self._add_kv_row("最近帧 HEX", self.tags["value_hex"], "-")

    def _build_plot_panel(self) -> None:
        with dpg.collapsing_header(label="实时曲线", default_open=True):
            self._create_plot("速度", "rad/s", self.tags["velocity_series"], "velocity_y_axis")
            self._create_plot("输出占空比", "duty", self.tags["output_series"], "output_y_axis")
            self._create_plot("位置", "rad", self.tags["position_series"], "position_y_axis")
            self._create_plot("母线电压", "V", self.tags["voltage_series"], "voltage_y_axis")

    def _build_log_panel(self) -> None:
        with dpg.collapsing_header(label="通信日志", default_open=True):
            dpg.add_input_text(tag=self.tags["log_text"], multiline=True, readonly=True, width=-1, height=280)
            dpg.add_button(label="清空日志", callback=lambda: self.clear_logs())

    def _add_kv_row(self, key: str, value_tag: str, default_value: str) -> None:
        with dpg.table_row():
            dpg.add_text(key)
            dpg.add_text(default_value, tag=value_tag, wrap=280)

    def _create_plot(self, title: str, y_label: str, series_tag: str, y_axis_tag: str) -> None:
        with dpg.plot(label=title, height=170, width=-1):
            dpg.add_plot_legend()
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="样本")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=y_label, tag=y_axis_tag)
            dpg.add_line_series([], [], label=title, parent=y_axis, tag=series_tag)
            dpg.set_axis_limits_auto(x_axis)
            dpg.set_axis_limits_auto(y_axis)

    def refresh_ports(self, select_port: str = "") -> None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        dpg.configure_item(self.tags["port_combo"], items=ports)
        chosen = select_port or (ports[0] if ports else "")
        dpg.set_value(self.tags["port_combo"], chosen)
        self._append_log("SYS", self.font_status)
        if ports:
            self._append_log("SYS", f"刷新串口列表: {', '.join(ports)}")
        else:
            self._append_log("SYS", "未发现可用串口")

    def connect(self) -> None:
        if self.worker is not None:
            self._append_log("SYS", "已有连接实例在运行")
            return
        port = dpg.get_value(self.tags["port_combo"]) or self.initial_port
        baudrate = int(dpg.get_value(self.tags["baud_input"]))
        if not port:
            self._append_log("ERR", "请选择串口后再连接")
            return
        self.worker = SerialWorker(port=port, baudrate=baudrate, on_event=self.post_event)
        dpg.set_value(self.tags["status_text"], f"连接中: {port} @ {baudrate}")
        self.worker.start()

    def disconnect(self) -> None:
        if self.worker is None:
            return
        worker = self.worker
        self.worker = None
        worker.stop()

    def clear_logs(self) -> None:
        self.log_lines.clear()
        dpg.set_value(self.tags["log_text"], "")

    def send_control_command(self) -> None:
        mode_label = dpg.get_value(self.tags["control_mode"])
        mode = next((item for item in self.config.control_modes if item.label == mode_label), None)
        if mode is None:
            self._append_log("ERR", f"未知控制模式: {mode_label}")
            return
        seq = self._next_seq()
        frame = CtrlCmdFrame(
            motor_id=int(dpg.get_value(self.tags["motor_id"])),
            ctrl_mode=mode.value,
            target=float(dpg.get_value(self.tags["control_target"])),
            ff0=float(dpg.get_value(self.tags["control_ff0"])),
            ff1=float(dpg.get_value(self.tags["control_ff1"])),
            seq=seq,
        )
        self._queue_frame(frame.to_bytes(), frame.to_summary(self.config.control_mode_labels), expected_seq=seq)

    def send_param_command(self) -> None:
        param_label = dpg.get_value(self.tags["param_choice"])
        param = next((item for item in self.config.param_defs if item.label == param_label), None)
        if param is None:
            self._append_log("ERR", f"未知参数项: {param_label}")
            return
        seq = self._next_seq()
        frame = ParamCmdFrame(
            motor_id=int(dpg.get_value(self.tags["motor_id"])),
            param_id=param.value,
            value=float(dpg.get_value(self.tags["param_value"])),
            seq=seq,
        )
        self._queue_frame(frame.to_bytes(), frame.to_summary(self.config.param_labels), expected_seq=seq)

    def send_raw_command(self) -> None:
        try:
            payload = parse_hex_string(str(dpg.get_value(self.tags["raw_hex"])))
        except ValueError as exc:
            self._append_log("ERR", str(exc))
            return
        self._queue_frame(payload, "原始帧发送", expected_seq=None)

    def _queue_frame(self, payload: bytes, summary: str, expected_seq: Optional[int]) -> None:
        if self.worker is None:
            self._append_log("ERR", "串口未连接，无法发送")
            return
        self.worker.send(payload, summary, expected_seq)

    def _next_seq(self) -> int:
        if dpg.get_value(self.tags["auto_seq"]):
            seq = self.sequence_counter & 0xFF
            self.sequence_counter = (self.sequence_counter + 1) & 0xFF
            dpg.set_value(self.tags["seq_input"], seq)
            return seq
        return int(dpg.get_value(self.tags["seq_input"])) & 0xFF

    def _process_events(self) -> None:
        while True:
            try:
                event_type, payload = self.events.get_nowait()
            except Empty:
                break

            if event_type == "connection":
                self._handle_connection_event(payload)
            elif event_type == "tx":
                self._handle_tx_event(payload)
            elif event_type == "rx":
                self._handle_rx_event(payload)
            elif event_type == "parse_error":
                self.state.error_count += 1
                self._append_log("ERR", str(payload))
                self._refresh_stats_only()
            elif event_type == "error":
                self._append_log("ERR", str(payload))

    def _handle_connection_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        connected = bool(info.get("connected"))
        message = str(info.get("message", ""))
        dpg.set_value(self.tags["status_text"], message or ("已连接" if connected else "未连接"))
        self._append_log("SYS", message or ("已连接" if connected else "已断开"))
        if not connected and self.worker is not None:
            self.worker = None

    def _handle_tx_event(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        summary = str(payload.get("summary", "发送数据"))
        raw = payload.get("payload", b"")
        expected_seq = payload.get("expected_seq")
        self._append_log("TX", f"{summary} | {format_hex(raw)}")
        if isinstance(expected_seq, int):
            self.pending_requests[expected_seq] = PendingRequest(
                seq=expected_seq,
                summary=summary,
                timestamp=time.time(),
            )

    def _handle_rx_event(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        frame = payload.get("frame")
        raw = payload.get("payload", b"")
        if not isinstance(frame, ReplyFrame):
            return

        self.state.frame_count += 1
        if self.state.last_seq is not None:
            expected_seq = (self.state.last_seq + 1) & 0xFF
            if frame.seq != expected_seq:
                self.state.lost_frames += (frame.seq - expected_seq) & 0xFF
        self.state.last_seq = frame.seq
        self.state.last_frame = frame
        self.state.histories["velocity"].append(frame.velocity)
        self.state.histories["output"].append(frame.output)
        self.state.histories["position"].append(frame.position)
        self.state.histories["voltage"].append(frame.voltage)
        self.plot_dirty = True

        pending = self.pending_requests.pop(frame.seq, None)
        if pending is not None:
            latency_ms = (time.time() - pending.timestamp) * 1000.0
            self.state.last_ack = f"seq={frame.seq} <- {pending.summary} ({latency_ms:.1f} ms)"
        else:
            self.state.last_ack = f"seq={frame.seq} 未匹配到待确认请求"

        self._append_log("RX", f"{frame.to_summary()} | {format_hex(raw)}")
        self._refresh_live_frame(raw)

    def _refresh_stats_only(self) -> None:
        dpg.set_value(self.tags["frame_count"], str(self.state.frame_count))
        dpg.set_value(self.tags["error_count"], str(self.state.error_count))
        dpg.set_value(self.tags["lost_frames"], str(self.state.lost_frames))
        dpg.set_value(self.tags["last_ack"], self.state.last_ack)

    def _refresh_live_frame(self, raw: bytes) -> None:
        frame = self.state.last_frame
        self._refresh_stats_only()
        if frame is None:
            return
        dpg.set_value(self.tags["value_motor_id"], str(frame.motor_id))
        dpg.set_value(self.tags["value_seq"], str(frame.seq))
        dpg.set_value(self.tags["value_status"], f"{parse_status(frame.status)} (0x{frame.status:02X})")
        dpg.set_value(self.tags["value_velocity"], f"{frame.velocity:.6f} rad/s")
        dpg.set_value(self.tags["value_output"], f"{frame.output:.6f}")
        dpg.set_value(self.tags["value_position"], f"{frame.position:.6f} rad")
        dpg.set_value(self.tags["value_voltage"], f"{frame.voltage:.6f} V")
        dpg.set_value(self.tags["value_hex"], format_hex(raw))

    def _update_plots(self) -> None:
        for key, series_tag in (
            ("velocity", self.tags["velocity_series"]),
            ("output", self.tags["output_series"]),
            ("position", self.tags["position_series"]),
            ("voltage", self.tags["voltage_series"]),
        ):
            values = list(self.state.histories[key])
            samples = list(range(len(values)))
            dpg.set_value(series_tag, [samples, values])

    def _append_log(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"[{timestamp}] [{level}] {message}")
        dpg.set_value(self.tags["log_text"], "\n".join(self.log_lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FOC Controller DearPyGui 上位机")
    parser.add_argument("port", nargs="?", default="", help="启动时默认串口，例如 COM3")
    parser.add_argument("baudrate", nargs="?", default=DEFAULT_BAUDRATE, type=int, help="启动时默认波特率")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = FOCGuiApp(initial_port=args.port, initial_baudrate=args.baudrate)
    app.run()


if __name__ == "__main__":
    main()
