#!/usr/bin/env python3
"""Fast Control Protocol lightweight host GUI."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import os
from pathlib import Path
import platform
from queue import Empty, Queue
import re
import shutil
import struct
import subprocess
import threading
import time
from typing import Deque, Optional, Sequence

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请执行: python -m pip install -r requirements.txt") from exc

try:
    from dearpygui import dearpygui as dpg
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少 DearPyGui，请执行: python -m pip install -r requirements.txt") from exc


FAST_CTRL_HEAD = 0x1545
FAST_STATUS_HEAD = 0x0C21
FAST_CTRL_SIZE = 7
FAST_STATUS_SIZE = 13

FAST_CMD_STOP = 0
FAST_CMD_SET_SPEED = 1
FAST_CMD_START_CALIBRATION = 2
FAST_CMD_SET_OUTPUT_LIMIT = 3
FAST_REQUEST_STATUS = 0x80

STATUS_MOTOR_ENABLED = 0x01
STATUS_COMMAND_CLAMPED = 0x02
STATUS_OUTPUT_SATURATED = 0x04
STATUS_ENCODER_FAULT = 0x08
STATUS_DRIVER_FAULT = 0x10
STATUS_BUS_UNDERVOLTAGE = 0x20
STATUS_MOTOR_FAULT = 0x40

SPEED_SCALE = 64.0
POSITION_SCALE = 6.283185307179586 / 65536.0
STATUS_TIMEOUT_SECONDS = 0.08
PLOT_HISTORY_SECONDS = 15.0

UI_SCALE_OVERRIDE_ENV = "FOC_HOST_UI_SCALE"
FONT_OVERRIDE_ENV = "FOC_HOST_FONT"

LINUX_FONT_CANDIDATES = (
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJKSC-Regular.otf"),
    Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
    Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
    Path("/usr/share/fonts/sarasa-gothic/Sarasa-Regular.ttc"),
    Path.home() / ".local/share/fonts/NotoSansCJK-Regular.ttc",
    Path.home() / ".local/share/fonts/SourceHanSansSC-Regular.otf",
)

WINDOWS_FONT_CANDIDATES = (
    Path("C:/Windows/Fonts/msyh.ttc"),
    Path("C:/Windows/Fonts/msyhbd.ttc"),
    Path("C:/Windows/Fonts/simhei.ttf"),
    Path("C:/Windows/Fonts/simsun.ttc"),
)

MACOS_FONT_CANDIDATES = (
    Path("/System/Library/Fonts/PingFang.ttc"),
    Path("/System/Library/Fonts/Hiragino Sans GB.ttc"),
)


def _run_text_command(args: Sequence[str], timeout: float = 1.5) -> Optional[str]:
    if not args or shutil.which(args[0]) is None:
        return None
    try:
        result = subprocess.run(
            list(args), capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


def _parse_last_float(raw: Optional[str]) -> Optional[float]:
    if not raw:
        return None
    values = re.findall(r"\d+(?:\.\d+)?", raw)
    if not values:
        return None
    try:
        return float(values[-1])
    except ValueError:
        return None


def _sanitize_scale(scale: float) -> float:
    return max(0.75, min(float(scale), 4.0))


def _detect_fontconfig_cjk_font() -> Optional[Path]:
    for pattern in (
        "sans-serif:lang=zh-cn:style=Regular",
        "sans:lang=zh-cn:style=Regular",
        "sans-serif:lang=zh-cn",
    ):
        output = _run_text_command(("fc-match", "-f", "%{file}\n", pattern))
        if output:
            path = Path(output.splitlines()[0]).expanduser()
            if path.is_file():
                return path
    return None


def find_cjk_font(override: str = "") -> Optional[Path]:
    candidates = []
    if override:
        candidates.append(Path(override).expanduser())
    if os.environ.get(FONT_OVERRIDE_ENV):
        candidates.append(Path(os.environ[FONT_OVERRIDE_ENV]).expanduser())

    system = platform.system()
    if system == "Windows":
        candidates.extend(WINDOWS_FONT_CANDIDATES)
    elif system == "Darwin":
        candidates.extend(MACOS_FONT_CANDIDATES)
    else:
        detected = _detect_fontconfig_cjk_font()
        if detected is not None:
            candidates.append(detected)
        candidates.extend(LINUX_FONT_CANDIDATES)

    seen = set()
    for path in candidates:
        path = path.expanduser()
        if path not in seen and path.is_file():
            return path
        seen.add(path)
    return None


def _detect_scale_from_tk() -> Optional[tuple[float, str]]:
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return None
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        dpi = float(root.winfo_fpixels("1i"))
        root.destroy()
    except Exception:
        return None
    if dpi <= 0.0:
        return None
    return _sanitize_scale(dpi / 96.0), f"Tk DPI({dpi:.1f})"


def detect_ui_scale(override: Optional[float] = None) -> tuple[float, str]:
    if override is not None and override > 0.0:
        scale = _sanitize_scale(override)
        return scale, f"命令行指定 {scale:.2f}x"

    env_scale = _parse_last_float(os.environ.get(UI_SCALE_OVERRIDE_ENV))
    if env_scale is not None and env_scale > 0.0:
        scale = _sanitize_scale(env_scale)
        return scale, f"{UI_SCALE_OVERRIDE_ENV}={scale:.2f}"

    qt_scale = _parse_last_float(os.environ.get("QT_SCALE_FACTOR"))
    if qt_scale is not None and qt_scale > 0.0:
        scale = _sanitize_scale(qt_scale)
        return scale, f"QT_SCALE_FACTOR={scale:.2f}"

    gdk_scale = _parse_last_float(os.environ.get("GDK_SCALE"))
    gdk_dpi_scale = _parse_last_float(os.environ.get("GDK_DPI_SCALE"))
    if gdk_scale is not None or gdk_dpi_scale is not None:
        scale = _sanitize_scale((gdk_scale or 1.0) * (gdk_dpi_scale or 1.0))
        return scale, f"GDK scale={scale:.2f}"

    qt_font_dpi = _parse_last_float(os.environ.get("QT_FONT_DPI"))
    if qt_font_dpi is not None and qt_font_dpi > 0.0:
        scale = _sanitize_scale(qt_font_dpi / 96.0)
        return scale, f"QT_FONT_DPI={qt_font_dpi:.1f}"

    if platform.system() == "Linux":
        scale_raw = _run_text_command(
            ("gsettings", "get", "org.gnome.desktop.interface", "scaling-factor")
        )
        text_raw = _run_text_command(
            ("gsettings", "get", "org.gnome.desktop.interface", "text-scaling-factor")
        )
        scale = (_parse_last_float(scale_raw) or 1.0) * (_parse_last_float(text_raw) or 1.0)
        if scale > 0.0:
            scale = _sanitize_scale(scale)
            return scale, f"gsettings({scale:.2f}x)"

    tk_scale = _detect_scale_from_tk()
    if tk_scale is not None:
        return tk_scale
    return 1.0, "默认 1.00x"


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def build_fast_ctrl(motor_id: int, command: int, value: int = 0, request_status: bool = False) -> bytes:
    control = command | (FAST_REQUEST_STATUS if request_status else 0)
    payload = struct.pack(">HBBH", FAST_CTRL_HEAD, motor_id & 0xFF, control, value & 0xFFFF)
    return payload + bytes((crc8(payload),))


def encode_speed(speed_rad_s: float) -> int:
    speed = max(-500.0, min(500.0, float(speed_rad_s)))
    return int(round(speed * SPEED_SCALE))


@dataclass(frozen=True)
class FastStatus:
    motor_id: int
    flags: int
    velocity: float
    current: float
    position: float
    bus_voltage: float
    timestamp: float


def decode_fast_status(frame: bytes) -> FastStatus:
    if len(frame) != FAST_STATUS_SIZE:
        raise ValueError("状态帧长度错误")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("状态帧 CRC 错误")

    head, motor_id, flags, velocity_raw, current_ma, position_raw, bus_mv, _ = struct.unpack(
        ">HBBhhHHB", frame
    )
    if head != FAST_STATUS_HEAD:
        raise ValueError("状态帧头错误")

    return FastStatus(
        motor_id=motor_id,
        flags=flags,
        velocity=velocity_raw / SPEED_SCALE,
        current=current_ma / 1000.0,
        position=position_raw * POSITION_SCALE,
        bus_voltage=bus_mv / 1000.0,
        timestamp=time.monotonic(),
    )


def status_text(flags: int) -> str:
    labels = []
    if flags & STATUS_MOTOR_ENABLED:
        labels.append("运行")
    else:
        labels.append("停止")
    if flags & STATUS_COMMAND_CLAMPED:
        labels.append("命令已限幅")
    if flags & STATUS_OUTPUT_SATURATED:
        labels.append("输出饱和")
    if flags & STATUS_ENCODER_FAULT:
        labels.append("编码器故障")
    if flags & STATUS_DRIVER_FAULT:
        labels.append("驱动故障")
    if flags & STATUS_BUS_UNDERVOLTAGE:
        labels.append("母线欠压")
    if flags & STATUS_MOTOR_FAULT:
        labels.append("电机故障")
    return " | ".join(labels)


class FastSerialWorker:
    def __init__(self, port: str, baudrate: int, motor_id: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.events: "Queue[tuple[str, object]]" = Queue()
        self.commands: "Queue[tuple[int, int, bool]]" = Queue()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._speed_enabled = False
        self._target_speed = 0.0
        self._command_hz = 100.0
        self._status_hz = 20.0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="fast-rs485", daemon=True)
        self._thread.start()

    def configure_stream(self, enabled: bool, target: float, command_hz: float, status_hz: float) -> None:
        with self._lock:
            self._speed_enabled = bool(enabled)
            self._target_speed = max(-500.0, min(500.0, float(target)))
            self._command_hz = max(1.0, min(500.0, float(command_hz)))
            self._status_hz = max(0.2, min(self._command_hz, float(status_hz)))

    def queue_command(self, command: int, value: int = 0, request_status: bool = False) -> None:
        self.commands.put((command, value, request_status))

    def stop_motor(self) -> None:
        with self._lock:
            self._speed_enabled = False
        self.queue_command(FAST_CMD_STOP, 0, True)

    def close(self) -> None:
        if self._thread is None:
            return
        with self._lock:
            self._speed_enabled = False
        while True:
            try:
                self.commands.get_nowait()
            except Empty:
                break
        self.commands.put((-1, 0, False))
        self._thread.join(timeout=0.4)
        if self._thread.is_alive():
            self._stop.set()
            self._thread.join(timeout=0.5)

    def _snapshot_stream(self) -> tuple[bool, float, float, float]:
        with self._lock:
            return self._speed_enabled, self._target_speed, self._command_hz, self._status_hz

    @staticmethod
    def _extract_status(buffer: bytearray) -> Optional[FastStatus]:
        header = bytes((FAST_STATUS_HEAD >> 8, FAST_STATUS_HEAD & 0xFF))
        while len(buffer) >= 2:
            index = buffer.find(header)
            if index < 0:
                if buffer[-1] == header[0]:
                    del buffer[:-1]
                else:
                    buffer.clear()
                return None
            if index:
                del buffer[:index]
            if len(buffer) < FAST_STATUS_SIZE:
                return None
            frame = bytes(buffer[:FAST_STATUS_SIZE])
            try:
                status = decode_fast_status(frame)
            except ValueError:
                del buffer[0]
                continue
            del buffer[:FAST_STATUS_SIZE]
            return status
        return None

    def _run(self) -> None:
        rx_buffer = bytearray()
        waiting_status = False
        status_deadline = 0.0
        next_command = time.monotonic()
        next_status = time.monotonic()

        try:
            port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0,
                write_timeout=0.05,
            )
            port.reset_input_buffer()
            port.reset_output_buffer()
            self.events.put(("connected", self.port))

            while not self._stop.is_set():
                now = time.monotonic()
                waiting = port.in_waiting
                if waiting:
                    rx_buffer.extend(port.read(waiting))
                    while True:
                        status = self._extract_status(rx_buffer)
                        if status is None:
                            break
                        if status.motor_id != self.motor_id:
                            continue
                        waiting_status = False
                        self.events.put(("status", status))

                if waiting_status and now >= status_deadline:
                    waiting_status = False
                    self.events.put(("timeout", "Fast Status 超时"))

                if not waiting_status:
                    try:
                        command, value, request_status = self.commands.get_nowait()
                    except Empty:
                        command = -2
                        value = 0
                        request_status = False

                    if command == -1:
                        port.write(build_fast_ctrl(self.motor_id, FAST_CMD_STOP))
                        port.flush()
                        break
                    if command >= 0:
                        frame = build_fast_ctrl(self.motor_id, command, value, request_status)
                        port.write(frame)
                        self.events.put(("tx", frame))
                        if request_status and command != FAST_CMD_START_CALIBRATION:
                            waiting_status = True
                            status_deadline = now + STATUS_TIMEOUT_SECONDS
                    else:
                        enabled, target, command_hz, status_hz = self._snapshot_stream()
                        if enabled and now >= next_command:
                            request_status = now >= next_status
                            frame = build_fast_ctrl(
                                self.motor_id,
                                FAST_CMD_SET_SPEED,
                                encode_speed(target),
                                request_status,
                            )
                            port.write(frame)
                            if request_status:
                                waiting_status = True
                                status_deadline = now + STATUS_TIMEOUT_SECONDS
                                next_status = now + 1.0 / status_hz
                            next_command = now + 1.0 / command_hz

                time.sleep(0.001)

        except (OSError, serial.SerialException) as exc:
            self.events.put(("error", str(exc)))
        finally:
            try:
                port.close()
            except (UnboundLocalError, OSError, serial.SerialException):
                pass
            self.events.put(("disconnected", self.port))


class FastHostApp:
    def __init__(
        self,
        initial_port: str,
        baudrate: int,
        motor_id: int,
        ui_scale_override: Optional[float] = None,
        font_override: str = "",
    ) -> None:
        self.initial_port = initial_port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.ui_scale, scale_source = detect_ui_scale(ui_scale_override)
        self.scale_status = f"UI 缩放 {self.ui_scale:.2f}x ({scale_source})"
        self.font_override = font_override
        self.font_status = "未加载中文字体"
        self.worker: Optional[FastSerialWorker] = None
        self.connected = False
        self.running = False
        self.start_time = time.monotonic()
        self.last_plot_refresh = 0.0
        self.history_t: Deque[float] = deque()
        self.history_speed: Deque[float] = deque()
        self.history_current: Deque[float] = deque()
        self.log_lines: Deque[str] = deque(maxlen=120)

    def run(self) -> None:
        dpg.create_context()
        self._load_font()
        self._build_theme()
        self._build_ui()
        self._log("SYS", self.scale_status)
        self._log("SYS", self.font_status)
        dpg.create_viewport(
            title="FOC Fast Control",
            width=self._px(1080),
            height=self._px(760),
            min_width=self._px(840),
            min_height=self._px(620),
        )
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main_window", True)

        try:
            while dpg.is_dearpygui_running():
                self._poll_worker()
                self._refresh_plot()
                dpg.render_dearpygui_frame()
        finally:
            if self.worker is not None:
                self.worker.close()
            dpg.destroy_context()

    def _px(self, value: int | float) -> int:
        if value < 0:
            return int(value)
        return max(1, int(round(float(value) * self.ui_scale)))

    def _scaled(self, value: int | float) -> float:
        return float(value) * self.ui_scale

    def _load_font(self) -> None:
        font_path = find_cjk_font(self.font_override)
        if font_path is None:
            self.font_status = (
                "未找到中文字体，可用 --font-file 或 "
                f"{FONT_OVERRIDE_ENV} 指定"
            )
            return
        with dpg.font_registry():
            with dpg.font(str(font_path), self._px(18)) as font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
                dpg.add_font_range(0x2000, 0x206F)
                dpg.add_font_range(0xFF00, 0xFFEF)
        dpg.bind_font(font)
        self.font_status = f"中文字体 {font_path}"

    def _build_theme(self) -> None:
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, self._scaled(14), self._scaled(14))
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, self._scaled(9), self._scaled(6))
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, self._scaled(9), self._scaled(8))
                dpg.add_theme_style(dpg.mvStyleVar_CellPadding, self._scaled(7), self._scaled(5))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, self._scaled(5))
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, self._scaled(6))
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (22, 27, 33, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (29, 36, 44, 255))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (38, 47, 57, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (48, 88, 112, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (62, 112, 140, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (43, 73, 88, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (232, 237, 241, 255))
        dpg.bind_theme(theme)

    def _build_ui(self) -> None:
        ports = self._serial_ports()
        selected_port = self.initial_port or (ports[0] if ports else "")
        with dpg.window(tag="main_window", no_title_bar=True):
            dpg.add_text("FOC Fast Control", color=(238, 244, 247))
            dpg.add_separator()

            with dpg.group(horizontal=True):
                dpg.add_combo(ports, default_value=selected_port, tag="port", width=self._px(250))
                dpg.add_button(label="刷新", callback=self._refresh_ports, width=self._px(70))
                dpg.add_input_int(label="波特率", default_value=self.baudrate, tag="baudrate", width=self._px(130))
                dpg.add_input_int(label="电机 ID", default_value=self.motor_id, tag="motor_id", width=self._px(90), min_value=0, max_value=255)
                dpg.add_button(label="连接", callback=self._toggle_connection, tag="connect_button", width=self._px(90))
                dpg.add_text("未连接", tag="connection_state", color=(185, 192, 199))

            dpg.add_spacer(height=self._px(5))
            with dpg.group(horizontal=True):
                with dpg.child_window(width=self._px(430), height=self._px(245), border=True):
                    dpg.add_text("控制")
                    dpg.add_input_float(
                        label="目标速度 (rad/s)",
                        default_value=0.0,
                        tag="target_speed",
                        width=self._px(190),
                        min_value=-500.0,
                        max_value=500.0,
                        min_clamped=True,
                        max_clamped=True,
                        format="%.3f",
                        callback=self._stream_settings_changed,
                    )
                    with dpg.group(horizontal=True):
                        dpg.add_input_float(label="下发 Hz", default_value=100.0, tag="command_hz", width=self._px(105), min_value=1.0, max_value=500.0, callback=self._stream_settings_changed)
                        dpg.add_input_float(label="状态 Hz", default_value=20.0, tag="status_hz", width=self._px(105), min_value=0.2, max_value=100.0, callback=self._stream_settings_changed)
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="运行", callback=self._start_motor, width=self._px(120))
                        dpg.add_button(label="停止", callback=self._stop_motor, width=self._px(120))
                    dpg.add_separator()
                    with dpg.group(horizontal=True):
                        dpg.add_input_float(label="动力限制 (A)", default_value=1.0, tag="output_limit", width=self._px(145), min_value=0.0, max_value=65.535)
                        dpg.add_button(label="设置", callback=self._set_output_limit, width=self._px(75))
                    dpg.add_button(label="启动编码器矫正", callback=self._show_calibration_confirm, width=self._px(190))

                with dpg.child_window(autosize_x=True, height=self._px(245), border=True):
                    dpg.add_text("实时状态")
                    with dpg.table(header_row=False, policy=dpg.mvTable_SizingStretchProp):
                        dpg.add_table_column()
                        dpg.add_table_column()
                        for label, tag, value in (
                            ("速度", "value_speed", "- rad/s"),
                            ("估算电流", "value_current", "- A"),
                            ("矫正位置", "value_position", "- rad"),
                            ("母线电压", "value_voltage", "- V"),
                        ):
                            with dpg.table_row():
                                dpg.add_text(label, color=(150, 163, 174))
                                dpg.add_text(value, tag=tag)
                    dpg.add_separator()
                    dpg.add_text("未收到状态", tag="status_flags", wrap=self._px(500))

            dpg.add_spacer(height=self._px(5))
            with dpg.plot(label="速度与估算电流", height=self._px(300), width=-1, anti_aliased=True):
                dpg.add_plot_legend()
                x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="时间 (s)", tag="plot_x")
                speed_axis = dpg.add_plot_axis(dpg.mvYAxis, label="速度 (rad/s)", tag="speed_axis")
                current_axis = dpg.add_plot_axis(dpg.mvYAxis2, label="电流 (A)", tag="current_axis")
                dpg.add_line_series([], [], label="速度", parent=speed_axis, tag="speed_series")
                dpg.add_line_series([], [], label="估算电流", parent=current_axis, tag="current_series")

            with dpg.collapsing_header(label="通信日志", default_open=False):
                dpg.add_input_text(tag="log", multiline=True, readonly=True, height=self._px(120), width=-1)

        with dpg.window(label="确认矫正", modal=True, show=False, tag="calibration_modal", width=self._px(360), height=self._px(135), no_resize=True):
            dpg.add_text("电机将进入自动矫正流程。")
            with dpg.group(horizontal=True):
                dpg.add_button(label="开始", callback=self._start_calibration, width=self._px(120))
                dpg.add_button(label="取消", callback=lambda *_: dpg.configure_item("calibration_modal", show=False), width=self._px(120))

    @staticmethod
    def _serial_ports() -> list[str]:
        return [port.device for port in list_ports.comports()]

    def _refresh_ports(self, *_: object) -> None:
        ports = self._serial_ports()
        dpg.configure_item("port", items=ports)
        if ports and dpg.get_value("port") not in ports:
            dpg.set_value("port", ports[0])

    def _toggle_connection(self, *_: object) -> None:
        if self.worker is not None:
            self.running = False
            self.worker.close()
            self.worker = None
            self.connected = False
            dpg.set_value("connection_state", "未连接")
            dpg.set_item_label("connect_button", "连接")
            self._log("SYS", "串口已断开")
            return

        port = str(dpg.get_value("port") or "").strip()
        if not port:
            self._log("ERR", "请选择串口")
            return
        self.worker = FastSerialWorker(
            port,
            int(dpg.get_value("baudrate")),
            int(dpg.get_value("motor_id")),
        )
        dpg.set_value("connection_state", "连接中")
        self.worker.start()

    def _stream_settings_changed(self, *_: object) -> None:
        if self.worker is not None:
            self.worker.configure_stream(
                self.running,
                float(dpg.get_value("target_speed")),
                float(dpg.get_value("command_hz")),
                float(dpg.get_value("status_hz")),
            )

    def _start_motor(self, *_: object) -> None:
        if not self._require_connection():
            return
        self.running = True
        self._stream_settings_changed()
        self._log("TX", f"开始速度流 target={float(dpg.get_value('target_speed')):.3f} rad/s")

    def _stop_motor(self, *_: object) -> None:
        if not self._require_connection():
            return
        self.running = False
        assert self.worker is not None
        self.worker.stop_motor()
        self._log("TX", "STOP")

    def _set_output_limit(self, *_: object) -> None:
        if not self._require_connection():
            return
        limit = max(0.0, min(65.535, float(dpg.get_value("output_limit"))))
        assert self.worker is not None
        self.worker.queue_command(FAST_CMD_SET_OUTPUT_LIMIT, int(round(limit * 1000.0)), True)
        self._log("TX", f"运行时动力限制 ±{limit:.3f} A")

    def _show_calibration_confirm(self, *_: object) -> None:
        if self._require_connection():
            dpg.configure_item("calibration_modal", show=True)

    def _start_calibration(self, *_: object) -> None:
        dpg.configure_item("calibration_modal", show=False)
        if not self._require_connection():
            return
        self.running = False
        assert self.worker is not None
        self.worker.configure_stream(False, 0.0, 100.0, 20.0)
        self.worker.queue_command(FAST_CMD_START_CALIBRATION)
        self._log("TX", "启动静默编码器矫正")

    def _require_connection(self) -> bool:
        if self.worker is None or not self.connected:
            self._log("ERR", "串口未连接")
            return False
        return True

    def _poll_worker(self) -> None:
        if self.worker is None:
            return
        worker = self.worker
        while True:
            try:
                kind, payload = worker.events.get_nowait()
            except Empty:
                break

            if kind == "connected":
                self.connected = True
                dpg.set_value("connection_state", f"已连接 {payload}")
                dpg.set_item_label("connect_button", "断开")
                self._log("SYS", f"已连接 {payload}")
            elif kind == "disconnected":
                self.connected = False
                self.running = False
                dpg.set_value("connection_state", "未连接")
                dpg.set_item_label("connect_button", "连接")
                self._log("SYS", "串口已断开")
                if self.worker is worker:
                    self.worker = None
            elif kind == "status":
                self._handle_status(payload)
            elif kind == "timeout":
                self._log("WARN", str(payload))
            elif kind == "error":
                self._log("ERR", str(payload))

    def _handle_status(self, status: FastStatus) -> None:
        dpg.set_value("value_speed", f"{status.velocity:.3f} rad/s")
        dpg.set_value("value_current", f"{status.current:.3f} A")
        dpg.set_value("value_position", f"{status.position:.5f} rad")
        dpg.set_value("value_voltage", f"{status.bus_voltage:.3f} V")
        dpg.set_value("status_flags", status_text(status.flags))

        elapsed = status.timestamp - self.start_time
        self.history_t.append(elapsed)
        self.history_speed.append(status.velocity)
        self.history_current.append(status.current)
        cutoff = elapsed - PLOT_HISTORY_SECONDS
        while self.history_t and self.history_t[0] < cutoff:
            self.history_t.popleft()
            self.history_speed.popleft()
            self.history_current.popleft()

    def _refresh_plot(self) -> None:
        now = time.monotonic()
        if now - self.last_plot_refresh < 1.0 / 30.0 or not self.history_t:
            return
        self.last_plot_refresh = now
        x = list(self.history_t)
        dpg.set_value("speed_series", [x, list(self.history_speed)])
        dpg.set_value("current_series", [x, list(self.history_current)])
        dpg.set_axis_limits("plot_x", max(0.0, x[-1] - PLOT_HISTORY_SECONDS), max(PLOT_HISTORY_SECONDS, x[-1]))
        dpg.fit_axis_data("speed_axis")
        dpg.fit_axis_data("current_axis")

    def _log(self, level: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"[{stamp}] [{level}] {message}")
        if dpg.does_item_exist("log"):
            dpg.set_value("log", "\n".join(self.log_lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FOC Fast Control 轻量上位机")
    parser.add_argument("port", nargs="?", default="", help="默认串口")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--motor-id", type=int, default=3)
    parser.add_argument("--ui-scale", type=float, default=None, help="覆盖自动UI缩放，例如1.5")
    parser.add_argument("--font-file", default="", help="指定中文字体文件")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    FastHostApp(
        args.port,
        args.baudrate,
        args.motor_id,
        ui_scale_override=args.ui_scale,
        font_override=args.font_file,
    ).run()


if __name__ == "__main__":
    main()
