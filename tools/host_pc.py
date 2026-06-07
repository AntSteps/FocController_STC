#!/usr/bin/env python3
"""FOC Controller DearPyGui 上位机。"""

from __future__ import annotations

import argparse
from collections import deque
import numpy as np
from dataclasses import dataclass, field
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import time
from typing import Callable, Deque, Dict, List, Optional

from protocol_codec import (
    CALIB_STATE_FAIL_BUSY,
    CALIB_STATE_FAIL_ENCODER,
    CALIB_STATE_FAIL_INTERNAL,
    CALIB_STATE_IDLE,
    CALIB_STATE_RUNNING,
    CALIB_STATE_SUCCESS,
    CTRL_MODE_SPEED,
    CTRL_MODE_STOP,
    REQUEST_KIND_CALIB,
    REQUEST_KIND_CTRL,
    REQUEST_KIND_PARAM_WRITE,
    REQUEST_KIND_QUERY,
    RESULT_BUSY,
    RESULT_CALIB_FAILED,
    RESULT_EEPROM_ERROR,
    RESULT_INTERNAL_ERROR,
    RESULT_INVALID_PARAM,
    RESULT_INVALID_VALUE,
    RESULT_OK,
    RESULT_UNSUPPORTED,
    format_hex,
    parse_status,
)
from protocol_service import (
    AckReply,
    CalibStatusReply,
    DEFAULT_BAUDRATE,
    DEFAULT_CALIB_POLL_HZ,
    DEFAULT_POLL_HZ,
    ParamSnapshotReply,
    ParamValueReply,
    ProtocolService,
    StateReply,
    list_serial_ports,
)

try:
    from dearpygui import dearpygui as dpg
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "缺少 DearPyGui，请先执行: python -m pip install -r requirements.txt"
    ) from exc


MAX_LOG_LINES = 300
DEFAULT_HISTORY_LENGTH = 500
PAN_THRESHOLD = 100  # 数据量超过此值后 X 轴开始平移滚动

WINDOWS_FONT_CANDIDATES = [
    Path("C:/Windows/Fonts/msyh.ttc"),
    Path("C:/Windows/Fonts/msyhbd.ttc"),
    Path("C:/Windows/Fonts/msyh.ttf"),
    Path("C:/Windows/Fonts/simhei.ttf"),
    Path("C:/Windows/Fonts/simsun.ttc"),
    Path("C:/Windows/Fonts/Deng.ttf"),
    Path("C:/Windows/Fonts/Dengb.ttf"),
]

LINUX_FONT_CANDIDATES = [
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJKSC-Regular.otf"),
    Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
    Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
    Path("/usr/share/fonts/truetype/arphic/ukai.ttc"),
    Path("/usr/share/fonts/truetype/arphic/uming.ttc"),
    Path("/usr/share/fonts/sarasa-gothic/Sarasa-Regular.ttc"),
    Path("/usr/share/fonts/MiSans-Regular.ttf"),
    Path("/usr/share/fonts/PingFang SC Regular.ttf"),
    Path("/usr/share/fonts/PingFang SC Bold.ttf"),
    Path.home() / ".local/share/fonts/NotoSansCJK-Regular.ttc",
    Path.home() / ".local/share/fonts/SourceHanSansSC-Regular.otf",
]

MACOS_FONT_CANDIDATES = [
    Path("/System/Library/Fonts/PingFang.ttc"),
    Path("/System/Library/Fonts/Hiragino Sans GB.ttc"),
    Path("/Library/Fonts/Arial Unicode.ttf"),
]

UI_SCALE_OVERRIDE_ENV = "FOC_HOST_UI_SCALE"
FONT_OVERRIDE_ENV = "FOC_HOST_FONT"


def _run_text_command(args: List[str], timeout: float = 1.5) -> Optional[str]:
    if not args or shutil.which(args[0]) is None:
        return None

    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    if result.returncode != 0:
        return None

    output = result.stdout.strip()
    return output or None


def _parse_last_float(raw: Optional[str]) -> Optional[float]:
    if not raw:
        return None

    matches = re.findall(r"\d+(?:\.\d+)?", raw)
    if not matches:
        return None

    try:
        return float(matches[-1])
    except ValueError:
        return None


def _sanitize_scale(scale: float) -> float:
    return max(0.75, min(scale, 4.0))


def _detect_fontconfig_cjk_font() -> Optional[Path]:
    for pattern in (
        "sans-serif:lang=zh-cn:style=Regular",
        "sans:lang=zh-cn:style=Regular",
        "sans-serif:lang=zh-cn",
        "sans:lang=zh-cn",
    ):
        output = _run_text_command(["fc-match", "-f", "%{file}\n", pattern])
        if not output:
            continue

        font_path = Path(output.splitlines()[0]).expanduser()
        if font_path.exists() and font_path.is_file():
            return font_path
    return None


def _detect_scale_from_gsettings() -> Optional[tuple[float, str]]:
    scale_raw = _run_text_command(
        ["gsettings", "get", "org.gnome.desktop.interface", "scaling-factor"]
    )
    text_scale_raw = _run_text_command(
        ["gsettings", "get", "org.gnome.desktop.interface", "text-scaling-factor"]
    )

    scale = _parse_last_float(scale_raw) or 1.0
    text_scale = _parse_last_float(text_scale_raw) or 1.0
    detected = scale * text_scale
    if detected <= 0.0:
        return None

    return _sanitize_scale(detected), f"gsettings({detected:.2f}x)"


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

    scale = dpi / 96.0
    return _sanitize_scale(scale), f"Tk DPI({dpi:.1f})"


def detect_ui_scale(override: Optional[float] = None) -> tuple[float, str]:
    if override is not None and override > 0.0:
        scale = _sanitize_scale(float(override))
        return scale, f"命令行指定缩放 {scale:.2f}x"

    override_env = _parse_last_float(os.environ.get(UI_SCALE_OVERRIDE_ENV))
    if override_env is not None and override_env > 0.0:
        scale = _sanitize_scale(override_env)
        return scale, f"{UI_SCALE_OVERRIDE_ENV}={scale:.2f}"

    qt_scale = _parse_last_float(os.environ.get("QT_SCALE_FACTOR"))
    if qt_scale is not None and qt_scale > 0.0:
        scale = _sanitize_scale(qt_scale)
        return scale, f"QT_SCALE_FACTOR={scale:.2f}"

    gdk_scale = _parse_last_float(os.environ.get("GDK_SCALE"))
    gdk_dpi_scale = _parse_last_float(os.environ.get("GDK_DPI_SCALE"))
    if gdk_scale is not None or gdk_dpi_scale is not None:
        scale = (gdk_scale or 1.0) * (gdk_dpi_scale or 1.0)
        scale = _sanitize_scale(scale)
        return scale, f"GDK scale={scale:.2f}"

    qt_font_dpi = _parse_last_float(os.environ.get("QT_FONT_DPI"))
    if qt_font_dpi is not None and qt_font_dpi > 0.0:
        scale = _sanitize_scale(qt_font_dpi / 96.0)
        return scale, f"QT_FONT_DPI={qt_font_dpi:.1f}"

    if platform.system() == "Linux":
        gsettings_scale = _detect_scale_from_gsettings()
        if gsettings_scale is not None:
            return gsettings_scale

    tk_scale = _detect_scale_from_tk()
    if tk_scale is not None:
        return tk_scale

    return 1.0, "默认缩放 1.00x"


def find_cjk_font(override: str = "") -> Optional[Path]:
    candidates: List[Path] = []
    if override:
        candidates.append(Path(override).expanduser())

    font_override = os.environ.get(FONT_OVERRIDE_ENV)
    if font_override:
        candidates.append(Path(font_override).expanduser())

    system = platform.system()
    if system == "Windows":
        candidates.extend(WINDOWS_FONT_CANDIDATES)
    elif system == "Darwin":
        candidates.extend(MACOS_FONT_CANDIDATES)
    else:
        fontconfig_font = _detect_fontconfig_cjk_font()
        if fontconfig_font is not None:
            candidates.append(fontconfig_font)
        candidates.extend(LINUX_FONT_CANDIDATES)

    seen: set[Path] = set()
    for font_path in candidates:
        normalized = font_path.expanduser()
        if normalized in seen:
            continue
        seen.add(normalized)
        if normalized.exists() and normalized.is_file():
            return normalized
    return None


@dataclass(frozen=True)
class ChoiceOption:
    value: int
    label: str
    description: str = ""


@dataclass
class QueuedCommand:
    summary: str
    callback: Callable[[], None]


@dataclass
class TelemetryState:
    reply_count: int = 0
    state_count: int = 0
    error_count: int = 0
    timeout_count: int = 0
    last_state: Optional[StateReply] = None
    last_ack_text: str = "等待命令结果"
    last_param_value: Optional[ParamValueReply] = None
    last_snapshot: Optional[ParamSnapshotReply] = None
    last_calib: Optional[CalibStatusReply] = None
    last_raw_hex: str = "-"
    histories: Dict[str, Deque[float]] = field(
        default_factory=lambda: {
            "velocity": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "output": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "position": deque(maxlen=DEFAULT_HISTORY_LENGTH),
            "voltage": deque(maxlen=DEFAULT_HISTORY_LENGTH),
        }
    )
    history_timestamps: Deque[float] = field(
        default_factory=lambda: deque(maxlen=DEFAULT_HISTORY_LENGTH)
    )


@dataclass
class AppConfig:
    control_modes: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(CTRL_MODE_STOP, "停止/休眠", "ctrl=0"),
            ChoiceOption(CTRL_MODE_SPEED, "速度模式", "ctrl=1"),
        ]
    )
    param_defs: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(0x01, "速度环 Kp"),
            ChoiceOption(0x02, "速度环 Ki"),
            ChoiceOption(0x03, "速度限制"),
            ChoiceOption(0x04, "电流限制"),
            ChoiceOption(0x05, "速度前馈增益"),
            ChoiceOption(0x06, "前馈使能"),
            ChoiceOption(0x10, "电机极对数"),
            ChoiceOption(0x11, "相电阻"),
            ChoiceOption(0x12, "相电感"),
            ChoiceOption(0x13, "电流环 Kp"),
            ChoiceOption(0x14, "电流环 Ki"),
            ChoiceOption(0x15, "速度输出上限"),
            ChoiceOption(0x16, "速度输出下限"),
            ChoiceOption(0x17, "速度滤波常数"),
            ChoiceOption(0x18, "加速度前馈"),
            ChoiceOption(0x19, "摩擦补偿"),
            ChoiceOption(0x1A, "速度环采样频率"),
            ChoiceOption(0x1B, "二自由度系数"),
            ChoiceOption(0x1C, "命令滤波常数"),
            ChoiceOption(0x1D, "PLL带宽"),
            ChoiceOption(0x1E, "PLL阻尼比"),
            ChoiceOption(0x20, "编码器偏移"),
            ChoiceOption(0x30, "校准极对数"),
            ChoiceOption(0x31, "校准方向"),
            ChoiceOption(0x32, "电角零位偏移"),
            ChoiceOption(0x33, "一次谐波幅值"),
            ChoiceOption(0x34, "一次谐波相位"),
            ChoiceOption(0x35, "二次谐波幅值"),
            ChoiceOption(0x36, "二次谐波相位"),
        ]
    )
    result_defs: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(RESULT_OK, "成功"),
            ChoiceOption(RESULT_BUSY, "忙碌"),
            ChoiceOption(RESULT_UNSUPPORTED, "不支持"),
            ChoiceOption(RESULT_INVALID_PARAM, "参数非法"),
            ChoiceOption(RESULT_INVALID_VALUE, "参数越界"),
            ChoiceOption(RESULT_EEPROM_ERROR, "EEPROM 错误"),
            ChoiceOption(RESULT_CALIB_FAILED, "校准失败"),
            ChoiceOption(RESULT_INTERNAL_ERROR, "内部错误"),
        ]
    )
    calib_state_defs: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(CALIB_STATE_IDLE, "空闲"),
            ChoiceOption(CALIB_STATE_RUNNING, "运行中"),
            ChoiceOption(CALIB_STATE_SUCCESS, "成功"),
            ChoiceOption(CALIB_STATE_FAIL_ENCODER, "编码器失败"),
            ChoiceOption(CALIB_STATE_FAIL_BUSY, "忙碌失败"),
            ChoiceOption(CALIB_STATE_FAIL_INTERNAL, "内部失败"),
        ]
    )
    ack_cmd_defs: List[ChoiceOption] = field(
        default_factory=lambda: [
            ChoiceOption(REQUEST_KIND_CTRL, "控制命令"),
            ChoiceOption(REQUEST_KIND_PARAM_WRITE, "参数写入"),
            ChoiceOption(REQUEST_KIND_QUERY, "查询命令"),
            ChoiceOption(REQUEST_KIND_CALIB, "校准命令"),
        ]
    )

    @property
    def control_mode_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.control_modes}

    @property
    def param_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.param_defs}

    @property
    def result_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.result_defs}

    @property
    def calib_state_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.calib_state_defs}

    @property
    def ack_cmd_labels(self) -> Dict[int, str]:
        return {item.value: item.label for item in self.ack_cmd_defs}


class FOCGuiApp:
    def __init__(
        self,
        initial_port: str = "",
        initial_baudrate: int = DEFAULT_BAUDRATE,
        ui_scale_override: Optional[float] = None,
        font_override: str = "",
    ) -> None:
        self.config = AppConfig()
        self.state = TelemetryState()
        self.initial_port = initial_port
        self.initial_baudrate = initial_baudrate
        self.ui_scale, scale_source = detect_ui_scale(ui_scale_override)
        self.scale_status = f"UI 缩放: {self.ui_scale:.2f}x ({scale_source})"
        self.font_override = font_override
        self.service = ProtocolService()
        self.log_lines: Deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.pending_commands: Deque[QueuedCommand] = deque()
        self.plot_dirty = False
        self.plot_paused = False
        self._cached_x: List[float] = []
        self._cached_x_len: int = 0
        self.font_status = "未加载中文字体"
        self.next_state_poll_at = 0.0
        self.next_calib_poll_at = 0.0

        self.tags = {
            "port_combo": "port_combo",
            "baud_input": "baud_input",
            "connect_button": "connect_button",
            "status_text": "status_text",
            "poll_enabled": "poll_enabled",
            "poll_hz": "poll_hz",
            "calib_poll_enabled": "calib_poll_enabled",
            "calib_poll_hz": "calib_poll_hz",
            "motor_id": "motor_id",
            "auto_seq": "auto_seq",
            "seq_input": "seq_input",
            "control_mode": "control_mode",
            "control_target": "control_target",
            "control_ff0": "control_ff0",
            "control_ff1": "control_ff1",
            "query_param_choice": "query_param_choice",
            "write_param_choice": "write_param_choice",
            "write_param_value": "write_param_value",
            "raw_hex": "raw_hex",
            "reply_count": "reply_count",
            "state_count": "state_count",
            "error_count": "error_count",
            "timeout_count": "timeout_count",
            "last_ack": "last_ack",
            "value_motor_id": "value_motor_id",
            "value_mode": "value_mode",
            "value_target": "value_target",
            "value_velocity": "value_velocity",
            "value_output": "value_output",
            "value_position": "value_position",
            "value_voltage": "value_voltage",
            "value_status": "value_status",
            "value_hex": "value_hex",
            "last_param": "last_param",
            "last_calib": "last_calib",
            "snapshot_text": "snapshot_text",
            "log_text": "log_text",
            "velocity_series": "velocity_series",
            "output_series": "output_series",
            "position_series": "position_series",
            "voltage_series": "voltage_series",
            "pause_plots": "pause_plots",
            "vel_x_axis": "vel_x_axis",
            "pos_x_axis": "pos_x_axis",
            "vol_x_axis": "vol_x_axis",
            "fft_series": "fft_series",
            "fft_x_axis": "fft_x_axis",
            "fft_y_axis": "fft_y_axis",
        }

    def run(self) -> None:
        dpg.create_context()
        self._setup_fonts()
        self._build_theme()
        self._build_ui()
        dpg.set_primary_window("main_window", True)
        self.refresh_ports(select_port=self.initial_port)
        dpg.set_value(self.tags["baud_input"], int(self.initial_baudrate))
        self._append_log("SYS", self.scale_status)
        self._append_log("SYS", self.font_status)
        dpg.create_viewport(
            title="FOC Controller Host",
            width=self._px(1560),
            height=self._px(960),
            min_width=self._px(1260),
            min_height=self._px(820),
        )
        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():
            self._process_events()
            self._run_pollers()
            if self.plot_dirty:
                self._update_plots()
                self.plot_dirty = False
            dpg.render_dearpygui_frame()

        self.disconnect()
        dpg.destroy_context()

    def _px(self, value: int | float) -> int:
        if value < 0:
            return int(value)
        return max(1, int(round(float(value) * self.ui_scale)))

    def _scaled(self, value: int | float) -> float:
        return float(value) * self.ui_scale

    def _setup_fonts(self) -> None:
        font_path = find_cjk_font(self.font_override)
        if font_path is None:
            self.font_status = (
                "未找到系统中文字体，界面可能出现乱码"
                f"；可用 --font-file 或 {FONT_OVERRIDE_ENV} 指定字体文件"
            )
            return

        with dpg.font_registry():
            with dpg.font(str(font_path), self._px(18)) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
                dpg.add_font_range(0x2000, 0x206F)
                dpg.add_font_range(0xFF00, 0xFFEF)
        dpg.bind_font(default_font)
        self.font_status = f"已加载中文字体: {font_path}"

    def _build_theme(self) -> None:
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(
                    dpg.mvStyleVar_WindowPadding,
                    self._scaled(12),
                    self._scaled(12),
                )
                dpg.add_theme_style(
                    dpg.mvStyleVar_FramePadding,
                    self._scaled(10),
                    self._scaled(6),
                )
                dpg.add_theme_style(
                    dpg.mvStyleVar_ItemSpacing,
                    self._scaled(10),
                    self._scaled(10),
                )
                dpg.add_theme_style(
                    dpg.mvStyleVar_CellPadding,
                    self._scaled(8),
                    self._scaled(6),
                )
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, self._scaled(10))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, self._scaled(8))
                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, self._scaled(10))
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
                with dpg.child_window(width=self._px(500), autosize_y=True, border=False):
                    self._build_connection_panel()
                    dpg.add_spacer(height=self._px(6))
                    self._build_command_panel()
                    dpg.add_spacer(height=self._px(6))
                    self._build_live_panel()
                with dpg.child_window(width=-1, autosize_y=True, border=False):
                    self._build_snapshot_panel()
                    dpg.add_spacer(height=self._px(6))
                    self._build_plot_panel()
                    dpg.add_spacer(height=self._px(6))
                    self._build_log_panel()

    def _build_connection_panel(self) -> None:
        with dpg.collapsing_header(label="串口连接", default_open=True):
            dpg.add_text("RS485 会话")
            with dpg.group(horizontal=True):
                dpg.add_combo(
                    items=[],
                    width=self._px(200),
                    tag=self.tags["port_combo"],
                    no_arrow_button=False,
                )
                dpg.add_button(label="刷新串口", callback=lambda: self.refresh_ports())
            dpg.add_input_int(
                label="波特率",
                tag=self.tags["baud_input"],
                default_value=self.initial_baudrate,
                step=100000,
                min_value=9600,
                min_clamped=True,
                width=self._px(200),
            )
            with dpg.group(horizontal=True):
                dpg.add_button(label="连接", tag=self.tags["connect_button"], callback=lambda: self.connect())
                dpg.add_button(label="断开", callback=lambda: self.disconnect())
            dpg.add_text("未连接", tag=self.tags["status_text"])
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="自动状态轮询", tag=self.tags["poll_enabled"], default_value=True)
                dpg.add_input_float(
                    label="Hz",
                    tag=self.tags["poll_hz"],
                    default_value=DEFAULT_POLL_HZ,
                    width=self._px(120),
                    format="%.2f",
                )
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="自动校准查询", tag=self.tags["calib_poll_enabled"], default_value=False)
                dpg.add_input_float(
                    label="Hz ",
                    tag=self.tags["calib_poll_hz"],
                    default_value=DEFAULT_CALIB_POLL_HZ,
                    width=self._px(120),
                    format="%.2f",
                )

    def _build_command_panel(self) -> None:
        with dpg.collapsing_header(label="命令面板", default_open=True):
            dpg.add_input_int(
                label="电机 ID",
                tag=self.tags["motor_id"],
                default_value=1,
                min_value=0,
                max_value=255,
                min_clamped=True,
                max_clamped=True,
                width=self._px(200),
            )
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="自动序列号", tag=self.tags["auto_seq"], default_value=True)
                dpg.add_input_int(
                    label="SEQ",
                    tag=self.tags["seq_input"],
                    default_value=0,
                    min_value=0,
                    max_value=255,
                    min_clamped=True,
                    max_clamped=True,
                    width=self._px(120),
                )

            with dpg.tab_bar():
                with dpg.tab(label="控制"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.control_modes],
                        default_value=self.config.control_modes[0].label,
                        label="控制模式",
                        tag=self.tags["control_mode"],
                        width=self._px(240),
                    )
                    dpg.add_input_float(
                        label="目标值",
                        tag=self.tags["control_target"],
                        default_value=0.0,
                        width=self._px(240),
                        format="%.6f",
                    )
                    dpg.add_input_float(
                        label="前馈 0",
                        tag=self.tags["control_ff0"],
                        default_value=0.0,
                        width=self._px(240),
                        format="%.6f",
                    )
                    dpg.add_input_float(
                        label="前馈 1",
                        tag=self.tags["control_ff1"],
                        default_value=0.0,
                        width=self._px(240),
                        format="%.6f",
                    )
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="发送控制命令", callback=lambda: self.send_control_command())
                        dpg.add_button(label="发送停止命令", callback=lambda: self.send_stop_command())

                with dpg.tab(label="查询"):
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="读取状态", callback=lambda: self.send_state_query())
                        dpg.add_button(label="读取快照", callback=lambda: self.send_snapshot_query())
                        dpg.add_button(label="读取校准状态", callback=lambda: self.send_calibration_query())
                    dpg.add_combo(
                        items=[item.label for item in self.config.param_defs],
                        default_value=self.config.param_defs[0].label,
                        label="读取参数",
                        tag=self.tags["query_param_choice"],
                        width=self._px(240),
                    )
                    dpg.add_button(label="读取选中参数", callback=lambda: self.send_param_query())

                with dpg.tab(label="参数写入"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.param_defs],
                        default_value=self.config.param_defs[0].label,
                        label="参数项",
                        tag=self.tags["write_param_choice"],
                        width=self._px(240),
                    )
                    dpg.add_input_float(
                        label="参数值",
                        tag=self.tags["write_param_value"],
                        default_value=0.0,
                        width=self._px(240),
                        format="%.6f",
                    )
                    dpg.add_button(label="写入参数", callback=lambda: self.send_param_write())

                with dpg.tab(label="校准"):
                    dpg.add_text(
                        "收到 ACK 后固件会后台执行校准，进度请看校准状态回复。",
                        wrap=self._px(420),
                    )
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="开始自动校准", callback=lambda: self.send_calibration_start())
                        dpg.add_button(label="刷新校准状态", callback=lambda: self.send_calibration_query())

                with dpg.tab(label="原始帧"):
                    dpg.add_text("用于协议联调或临时验证未建模的下行帧。")
                    dpg.add_input_text(
                        label="HEX",
                        tag=self.tags["raw_hex"],
                        multiline=True,
                        width=-1,
                        height=self._px(140),
                        hint="例如: 13 45 01 01 00 01 12",
                    )
                    dpg.add_button(label="发送原始帧", callback=lambda: self.send_raw_command())

    def _build_live_panel(self) -> None:
        with dpg.collapsing_header(label="最近状态", default_open=True):
            with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp):
                dpg.add_table_column()
                dpg.add_table_column()
                self._add_kv_row("回复计数", self.tags["reply_count"], "0")
                self._add_kv_row("状态回复", self.tags["state_count"], "0")
                self._add_kv_row("解析错误", self.tags["error_count"], "0")
                self._add_kv_row("请求超时", self.tags["timeout_count"], "0")
                self._add_kv_row("最近匹配", self.tags["last_ack"], "等待命令结果")
                self._add_kv_row("电机 ID", self.tags["value_motor_id"], "-")
                self._add_kv_row("控制模式", self.tags["value_mode"], "-")
                self._add_kv_row("目标值", self.tags["value_target"], "-")
                self._add_kv_row("速度", self.tags["value_velocity"], "-")
                self._add_kv_row("输出", self.tags["value_output"], "-")
                self._add_kv_row("位置", self.tags["value_position"], "-")
                self._add_kv_row("电压", self.tags["value_voltage"], "-")
                self._add_kv_row("健康状态", self.tags["value_status"], "-")
                self._add_kv_row("最近参数", self.tags["last_param"], "-")
                self._add_kv_row("校准状态", self.tags["last_calib"], "-")
                self._add_kv_row("最近帧 HEX", self.tags["value_hex"], "-")

    def _build_snapshot_panel(self) -> None:
        with dpg.collapsing_header(label="参数快照", default_open=True):
            dpg.add_input_text(
                tag=self.tags["snapshot_text"],
                multiline=True,
                readonly=True,
                width=-1,
                height=self._px(260),
                default_value="等待参数快照...",
            )

    def _build_plot_panel(self) -> None:
        with dpg.collapsing_header(label="实时曲线", default_open=True):
            dpg.add_checkbox(
                label="暂停绘图",
                tag=self.tags["pause_plots"],
                default_value=False,
                callback=lambda s, v: setattr(self, "plot_paused", v),
            )
            self._create_dual_plot(
                "速度环跟踪",
                "速度 (rad/s)",
                "输出 (duty)",
                self.tags["velocity_series"],
                self.tags["output_series"],
                "vel_y_axis",
                "out_y_axis",
                x_axis_tag=self.tags["vel_x_axis"],
                height=260,
            )
            self._create_plot(
                "位置", "rad", self.tags["position_series"], "position_y_axis",
                x_axis_tag=self.tags["pos_x_axis"], height=150,
            )
            self._create_plot(
                "母线电压", "V", self.tags["voltage_series"], "voltage_y_axis",
                x_axis_tag=self.tags["vol_x_axis"], height=130,
            )
            # 速度 FFT 频谱图
            with dpg.plot(
                label="速度频谱 (FFT)", height=self._px(200), width=-1,
                anti_aliased=True, crosshairs=True,
            ):
                dpg.add_plot_legend()
                fft_x = dpg.add_plot_axis(dpg.mvXAxis, label="频率 (Hz)", tag=self.tags["fft_x_axis"])
                fft_y = dpg.add_plot_axis(dpg.mvYAxis, label="幅值", tag=self.tags["fft_y_axis"])
                dpg.add_line_series([], [], label="速度 FFT", parent=fft_y, tag=self.tags["fft_series"])
                dpg.set_axis_limits_auto(fft_y)

    def _build_log_panel(self) -> None:
        with dpg.collapsing_header(label="通信日志", default_open=True):
            dpg.add_input_text(
                tag=self.tags["log_text"],
                multiline=True,
                readonly=True,
                width=-1,
                height=self._px(300),
            )
            dpg.add_button(label="清空日志", callback=lambda: self.clear_logs())

    def _add_kv_row(self, key: str, value_tag: str, default_value: str) -> None:
        with dpg.table_row():
            dpg.add_text(key)
            dpg.add_text(default_value, tag=value_tag, wrap=self._px(320))

    def _create_plot(
        self,
        title: str,
        y_label: str,
        series_tag: str,
        y_axis_tag: str,
        x_axis_tag: str = "",
        height: int = 150,
    ) -> None:
        with dpg.plot(
            label=title, height=self._px(height), width=-1, anti_aliased=True, crosshairs=True
        ):
            dpg.add_plot_legend()
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="时间 (s)", tag=x_axis_tag or 0)
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=y_label, tag=y_axis_tag)
            dpg.add_line_series([], [], label=title, parent=y_axis, tag=series_tag)
            dpg.set_axis_limits_auto(y_axis)

    def _create_dual_plot(
        self,
        title: str,
        y1_label: str,
        y2_label: str,
        tag1: str,
        tag2: str,
        axis1_tag: str,
        axis2_tag: str,
        x_axis_tag: str = "",
        height: int = 260,
    ) -> None:
        with dpg.plot(
            label=title, height=self._px(height), width=-1, anti_aliased=True, crosshairs=True
        ):
            dpg.add_plot_legend()
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="时间 (s)", tag=x_axis_tag or 0)
            y1_axis = dpg.add_plot_axis(dpg.mvYAxis, label=y1_label, tag=axis1_tag)
            dpg.add_line_series([], [], label=y1_label, parent=y1_axis, tag=tag1)
            y2_axis = dpg.add_plot_axis(dpg.mvYAxis2, label=y2_label, tag=axis2_tag)
            dpg.add_line_series([], [], label=y2_label, parent=y2_axis, tag=tag2)
            dpg.set_axis_limits_auto(y1_axis)
            dpg.set_axis_limits_auto(y2_axis)

    def refresh_ports(self, select_port: str = "") -> None:
        ports = list_serial_ports()
        dpg.configure_item(self.tags["port_combo"], items=ports)

        current_value = str(dpg.get_value(self.tags["port_combo"]) or "")
        chosen = select_port or current_value
        if chosen not in ports:
            chosen = ports[0] if ports else ""
        dpg.set_value(self.tags["port_combo"], chosen)

        if ports:
            self._append_log("SYS", f"刷新串口列表: {', '.join(ports)}")
        else:
            self._append_log("SYS", "未发现可用串口")

    def connect(self) -> None:
        port = str(dpg.get_value(self.tags["port_combo"]) or self.initial_port)
        baudrate = int(dpg.get_value(self.tags["baud_input"]))
        if not port:
            self._append_log("ERR", "请选择串口后再连接")
            return

        self.pending_commands.clear()
        self.next_state_poll_at = time.monotonic()
        self.next_calib_poll_at = time.monotonic()
        dpg.set_value(self.tags["status_text"], f"连接中: {port} @ {baudrate}")
        self.service.connect(port=port, baudrate=baudrate)

    def disconnect(self) -> None:
        self.pending_commands.clear()
        self.service.disconnect()

    def clear_logs(self) -> None:
        self.log_lines.clear()
        dpg.set_value(self.tags["log_text"], "")

    def _find_choice_by_label(self, options: List[ChoiceOption], label: str) -> Optional[ChoiceOption]:
        for item in options:
            if item.label == label:
                return item
        return None

    def _selected_motor_id(self) -> int:
        return int(dpg.get_value(self.tags["motor_id"])) & 0xFF

    def _next_seq(self) -> int:
        auto_seq = bool(dpg.get_value(self.tags["auto_seq"]))
        manual_seq = int(dpg.get_value(self.tags["seq_input"])) & 0xFF
        seq = self.service.allocate_seq(auto_enabled=auto_seq, manual_seq=manual_seq)
        if auto_seq:
            dpg.set_value(self.tags["seq_input"], seq)
        return seq

    def send_control_command(self) -> None:
        mode_label = str(dpg.get_value(self.tags["control_mode"]))
        mode = self._find_choice_by_label(self.config.control_modes, mode_label)
        if mode is None:
            self._append_log("ERR", f"未知控制模式: {mode_label}")
            return

        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        target = float(dpg.get_value(self.tags["control_target"]))
        ff0 = float(dpg.get_value(self.tags["control_ff0"]))
        ff1 = float(dpg.get_value(self.tags["control_ff1"]))
        summary = f"控制命令 ID={motor_id} seq={seq} mode={mode.label} target={target:.3f}"
        self._schedule_user_command(
            summary,
            lambda motor_id=motor_id, ctrl_mode=mode.value, target=target, ff0=ff0, ff1=ff1, seq=seq: (
                self.service.send_control(
                    motor_id=motor_id,
                    ctrl_mode=ctrl_mode,
                    target=target,
                    ff0=ff0,
                    ff1=ff1,
                    seq=seq,
                )
            ),
        )

    def send_stop_command(self) -> None:
        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        summary = f"停止命令 ID={motor_id} seq={seq}"
        self._schedule_user_command(
            summary,
            lambda motor_id=motor_id, seq=seq: self.service.send_control(
                motor_id=motor_id,
                ctrl_mode=CTRL_MODE_STOP,
                target=0.0,
                ff0=0.0,
                ff1=0.0,
                seq=seq,
            ),
        )

    def send_param_write(self) -> None:
        param_label = str(dpg.get_value(self.tags["write_param_choice"]))
        param = self._find_choice_by_label(self.config.param_defs, param_label)
        if param is None:
            self._append_log("ERR", f"未知参数项: {param_label}")
            return

        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        value = float(dpg.get_value(self.tags["write_param_value"]))
        summary = f"参数写入 ID={motor_id} seq={seq} param={param.label} value={value:.6f}"
        self._schedule_user_command(
            summary,
            lambda motor_id=motor_id, param_id=param.value, value=value, seq=seq: self.service.send_param_write(
                motor_id=motor_id,
                param_id=param_id,
                value=value,
                seq=seq,
            ),
        )

    def send_state_query(self) -> None:
        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        self._schedule_user_command(
            f"读取状态 ID={motor_id} seq={seq}",
            lambda motor_id=motor_id, seq=seq: self.service.query_state(
                motor_id=motor_id,
                seq=seq,
            ),
        )

    def send_param_query(self) -> None:
        param_label = str(dpg.get_value(self.tags["query_param_choice"]))
        param = self._find_choice_by_label(self.config.param_defs, param_label)
        if param is None:
            self._append_log("ERR", f"未知参数项: {param_label}")
            return

        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        self._schedule_user_command(
            f"读取参数 ID={motor_id} seq={seq} param={param.label}",
            lambda motor_id=motor_id, param_id=param.value, seq=seq: self.service.query_param(
                motor_id=motor_id,
                param_id=param_id,
                seq=seq,
            ),
        )

    def send_snapshot_query(self) -> None:
        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        self._schedule_user_command(
            f"读取快照 ID={motor_id} seq={seq}",
            lambda motor_id=motor_id, seq=seq: self.service.query_snapshot(
                motor_id=motor_id,
                seq=seq,
            ),
        )

    def send_calibration_query(self) -> None:
        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        self._schedule_user_command(
            f"读取校准状态 ID={motor_id} seq={seq}",
            lambda motor_id=motor_id, seq=seq: self.service.query_calibration_status(
                motor_id=motor_id,
                seq=seq,
            ),
        )

    def send_calibration_start(self) -> None:
        motor_id = self._selected_motor_id()
        seq = self._next_seq()
        self._schedule_user_command(
            f"启动校准 ID={motor_id} seq={seq}",
            lambda motor_id=motor_id, seq=seq: self.service.start_calibration(
                motor_id=motor_id,
                seq=seq,
            ),
        )

    def send_raw_command(self) -> None:
        raw_text = str(dpg.get_value(self.tags["raw_hex"]))
        self._schedule_user_command(
            "发送原始帧",
            lambda raw_text=raw_text: self.service.send_raw_hex(raw_text),
        )

    def _schedule_user_command(self, summary: str, callback: Callable[[], None]) -> None:
        if self.pending_commands:
            self.pending_commands.append(QueuedCommand(summary=summary, callback=callback))
            self._append_log("SYS", f"手动命令已排队({len(self.pending_commands)}): {summary}")
            return

        if self.service.is_connected and self.service.is_waiting_reply:
            self.pending_commands.append(QueuedCommand(summary=summary, callback=callback))
            self._append_log("SYS", f"总线忙，手动命令已排队: {summary}")
            return

        callback()

    def _dispatch_pending_user_command(self) -> bool:
        if not self.pending_commands:
            return False

        queued = self.pending_commands.popleft()
        self._append_log("SYS", f"优先发送手动命令: {queued.summary}")
        queued.callback()
        return True

    def _process_events(self) -> None:
        for event_type, payload in self.service.poll_events():
            if event_type == "connection":
                self._handle_connection_event(payload)
            elif event_type == "tx":
                self._handle_tx_event(payload)
            elif event_type == "rx":
                self._handle_rx_event(payload)
            elif event_type == "timeout":
                self._handle_timeout_event(payload)
            elif event_type == "parse_error":
                self.state.error_count += 1
                self._append_log("ERR", str(payload))
                self._refresh_stats_only()
            elif event_type == "error":
                self._append_log("ERR", str(payload))

    def _run_pollers(self) -> None:
        if not self.service.is_connected:
            return

        if self.service.is_waiting_reply:
            return

        if self._dispatch_pending_user_command():
            return

        now = time.monotonic()
        calib_running = (
            self.state.last_calib is not None and self.state.last_calib.calib_state == CALIB_STATE_RUNNING
        )

        if dpg.get_value(self.tags["poll_enabled"]) and (not calib_running):
            state_hz = max(0.2, float(dpg.get_value(self.tags["poll_hz"])))
            if now >= self.next_state_poll_at:
                self.service.query_state(
                    motor_id=self._selected_motor_id(),
                    seq=self._next_seq(),
                )
                self.next_state_poll_at = now + (1.0 / state_hz)

        if dpg.get_value(self.tags["calib_poll_enabled"]) or calib_running:
            calib_hz = max(0.2, float(dpg.get_value(self.tags["calib_poll_hz"])))
            if now >= self.next_calib_poll_at:
                self.service.query_calibration_status(
                    motor_id=self._selected_motor_id(),
                    seq=self._next_seq(),
                )
                self.next_calib_poll_at = now + (1.0 / calib_hz)

    def _handle_connection_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        connected = bool(info.get("connected"))
        message = str(info.get("message", ""))

        if not connected:
            self.pending_commands.clear()

        dpg.set_value(self.tags["status_text"], message or ("已连接" if connected else "未连接"))
        self._append_log("SYS", message or ("已连接" if connected else "已断开"))

    def _handle_tx_event(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return

        summary = str(payload.get("summary", "发送数据"))
        raw = payload.get("payload", b"")

        self._append_log("TX", f"{summary} | {format_hex(raw)}")

    def _handle_timeout_event(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return

        seq = int(payload.get("seq", 0))
        summary = str(payload.get("summary", "请求"))
        elapsed_ms = float(payload.get("elapsed_ms", 0.0))
        self.state.timeout_count += 1
        self.state.last_ack_text = f"seq={seq} 请求超时: {summary} ({elapsed_ms:.1f} ms)"
        self._append_log("ERR", self.state.last_ack_text)
        self._refresh_stats_only()

    def _reply_to_summary(self, reply: object) -> str:
        if isinstance(reply, AckReply):
            cmd_name = self.config.ack_cmd_labels.get(reply.cmd_type, f"命令 {reply.cmd_type}")
            result_name = self.config.result_labels.get(reply.result_code, f"结果 {reply.result_code}")
            return f"ACK ID={reply.motor_id} seq={reply.seq} cmd={cmd_name} result={result_name}"
        if isinstance(reply, StateReply):
            mode_name = self.config.control_mode_labels.get(reply.control_mode, f"模式 {reply.control_mode}")
            return (
                f"STATE ID={reply.motor_id} seq={reply.seq} mode={mode_name} "
                f"target={reply.target:.3f} vel={reply.velocity:.3f} "
                f"out={reply.output:.3f} pos={reply.position:.3f} "
                f"Vbus={reply.voltage:.3f} status=0x{reply.health_status:02X}"
                f"({parse_status(reply.health_status)})"
            )
        if isinstance(reply, ParamValueReply):
            param_name = self.config.param_labels.get(reply.param_id, f"参数 {reply.param_id}")
            return f"PARAM ID={reply.motor_id} seq={reply.seq} {param_name}={reply.value:.6f}"
        if isinstance(reply, ParamSnapshotReply):
            return (
                f"SNAPSHOT ID={reply.motor_id} seq={reply.seq} "
                f"version={reply.snapshot_version} pole_pairs={reply.snapshot.motor_pole_pairs} "
                f"vel_limit={reply.snapshot.velocity_limit:.3f}"
            )
        if isinstance(reply, CalibStatusReply):
            state_name = self.config.calib_state_labels.get(reply.calib_state, f"状态 {reply.calib_state}")
            return (
                f"CALIB ID={reply.motor_id} seq={reply.seq} "
                f"state={state_name} progress={reply.progress}%"
            )
        return "未知回复"

    def _handle_rx_event(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return

        reply = payload.get("reply")
        raw = payload.get("payload", b"")
        if not isinstance(reply, (AckReply, StateReply, ParamValueReply, ParamSnapshotReply, CalibStatusReply)):
            return

        self.state.reply_count += 1
        self.state.last_raw_hex = format_hex(raw)
        summary = self._reply_to_summary(reply)
        matched_summary = payload.get("matched_summary")
        latency_ms = payload.get("latency_ms")

        if matched_summary is not None and latency_ms is not None:
            self.state.last_ack_text = (
                f"seq={reply.seq} <- {matched_summary} ({float(latency_ms):.1f} ms)"
            )
        else:
            self.state.last_ack_text = f"seq={reply.seq} 未匹配待确认请求"

        if isinstance(reply, AckReply):
            self.state.last_ack_text = f"{self.state.last_ack_text} | {summary}"
        elif isinstance(reply, StateReply):
            self.state.state_count += 1
            self.state.last_state = reply
            self.state.histories["velocity"].append(reply.velocity)
            self.state.histories["output"].append(reply.output)
            self.state.histories["position"].append(reply.position)
            self.state.histories["voltage"].append(reply.voltage)
            self.state.history_timestamps.append(time.time())
            self.plot_dirty = True
        elif isinstance(reply, ParamValueReply):
            self.state.last_param_value = reply
        elif isinstance(reply, ParamSnapshotReply):
            self.state.last_snapshot = reply
        elif isinstance(reply, CalibStatusReply):
            self.state.last_calib = reply

        self._append_log("RX", f"{summary} | {format_hex(raw)}")
        self._refresh_live_values()

    def _refresh_stats_only(self) -> None:
        dpg.set_value(self.tags["reply_count"], str(self.state.reply_count))
        dpg.set_value(self.tags["state_count"], str(self.state.state_count))
        dpg.set_value(self.tags["error_count"], str(self.state.error_count))
        dpg.set_value(self.tags["timeout_count"], str(self.state.timeout_count))
        dpg.set_value(self.tags["last_ack"], self.state.last_ack_text)

    def _refresh_live_values(self) -> None:
        state_reply = self.state.last_state
        param_reply = self.state.last_param_value
        calib_reply = self.state.last_calib
        snapshot_reply = self.state.last_snapshot

        self._refresh_stats_only()

        if state_reply is not None:
            dpg.set_value(self.tags["value_motor_id"], str(state_reply.motor_id))
            dpg.set_value(
                self.tags["value_mode"],
                self.config.control_mode_labels.get(state_reply.control_mode, f"模式 {state_reply.control_mode}"),
            )
            dpg.set_value(self.tags["value_target"], f"{state_reply.target:.6f}")
            dpg.set_value(self.tags["value_velocity"], f"{state_reply.velocity:.6f} rad/s")
            dpg.set_value(self.tags["value_output"], f"{state_reply.output:.6f}")
            dpg.set_value(self.tags["value_position"], f"{state_reply.position:.6f} rad")
            dpg.set_value(self.tags["value_voltage"], f"{state_reply.voltage:.6f} V")
            dpg.set_value(
                self.tags["value_status"],
                f"0x{state_reply.health_status:02X} ({parse_status(state_reply.health_status)})",
            )

        if param_reply is not None:
            param_label = self.config.param_labels.get(param_reply.param_id, f"参数 {param_reply.param_id}")
            dpg.set_value(self.tags["last_param"], f"{param_label} = {param_reply.value:.6f}")

        if calib_reply is not None:
            calib_label = self.config.calib_state_labels.get(calib_reply.calib_state, str(calib_reply.calib_state))
            dpg.set_value(self.tags["last_calib"], f"{calib_label} ({calib_reply.progress}%)")

        dpg.set_value(self.tags["value_hex"], self.state.last_raw_hex)

        if snapshot_reply is not None:
            lines = [
                f"motor_id = {snapshot_reply.motor_id}",
                f"snapshot_version = {snapshot_reply.snapshot_version}",
            ]
            lines.extend(snapshot_reply.snapshot.to_lines())
            dpg.set_value(self.tags["snapshot_text"], "\n".join(lines))

    def _update_plots(self) -> None:
        if self.plot_paused:
            return
        n = len(self.state.histories["velocity"])
        if n != self._cached_x_len:
            hz = max(0.2, float(dpg.get_value(self.tags["poll_hz"])))
            dt = 1.0 / hz
            self._cached_x = [i * dt for i in range(n)]
            self._cached_x_len = n
        x = self._cached_x[:n]

        # X 轴平移逻辑：数据量超过阈值后只显示最近 PAN_THRESHOLD 个点
        if n > PAN_THRESHOLD:
            x_min = x[n - PAN_THRESHOLD]
            x_max = x[-1]
        else:
            x_min = 0.0
            x_max = x[-1] if x else 1.0

        for key, series_tag in (
            ("velocity", self.tags["velocity_series"]),
            ("output", self.tags["output_series"]),
            ("position", self.tags["position_series"]),
            ("voltage", self.tags["voltage_series"]),
        ):
            values = list(self.state.histories[key])
            dpg.set_value(series_tag, [x[: len(values)], values])

        # 更新各图 X 轴范围
        for x_axis_tag in (self.tags["vel_x_axis"], self.tags["pos_x_axis"], self.tags["vol_x_axis"]):
            dpg.set_axis_limits(x_axis_tag, x_min, x_max)

        # 速度 FFT 频谱（使用真实采样时间）
        vel_data = list(self.state.histories["velocity"])
        ts_data = list(self.state.history_timestamps)
        if len(vel_data) >= 8 and len(ts_data) == len(vel_data):
            arr = np.array(vel_data, dtype=np.float64)
            arr = arr - np.mean(arr)  # 去直流

            # 计算真实平均采样间隔
            ts = np.array(ts_data, dtype=np.float64)
            dt_mean = np.mean(np.diff(ts))
            if dt_mean > 0:
                spectrum = np.abs(np.fft.rfft(arr))
                freqs = np.fft.rfftfreq(len(arr), d=dt_mean)
                dpg.set_value(self.tags["fft_series"], [freqs.tolist(), spectrum.tolist()])
                dpg.set_axis_limits(self.tags["fft_x_axis"], freqs[1] if len(freqs) > 1 else 0, freqs[-1])

    def _append_log(self, level: str, message: str) -> None:
        now = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime(now))
        self.log_lines.append(f"[{timestamp}.{int((now - int(now)) * 1000):03d}] [{level}] {message}")
        dpg.set_value(self.tags["log_text"], "\n".join(self.log_lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FOC Controller DearPyGui 上位机")
    parser.add_argument("port", nargs="?", default="", help="启动时默认串口，例如 COM3")
    parser.add_argument(
        "baudrate",
        nargs="?",
        default=DEFAULT_BAUDRATE,
        type=int,
        help="启动时默认波特率",
    )
    parser.add_argument(
        "--ui-scale",
        type=float,
        default=None,
        help=f"覆盖界面缩放，Linux 高 DPI 环境可手动指定，例如 1.50；也可使用 {UI_SCALE_OVERRIDE_ENV}",
    )
    parser.add_argument(
        "--font-file",
        default="",
        help=f"指定中文字体文件路径；也可使用 {FONT_OVERRIDE_ENV}",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = FOCGuiApp(
        initial_port=args.port,
        initial_baudrate=args.baudrate,
        ui_scale_override=args.ui_scale,
        font_override=args.font_file,
    )
    app.run()


if __name__ == "__main__":
    main()
