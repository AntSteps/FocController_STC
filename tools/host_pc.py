#!/usr/bin/env python3
"""FOC Controller DearPyGui 上位机。"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Deque, Dict, List, Optional

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

WINDOWS_FONT_CANDIDATES = [
    Path("C:/Windows/Fonts/msyh.ttc"),
    Path("C:/Windows/Fonts/msyhbd.ttc"),
    Path("C:/Windows/Fonts/msyh.ttf"),
    Path("C:/Windows/Fonts/simhei.ttf"),
    Path("C:/Windows/Fonts/simsun.ttc"),
    Path("C:/Windows/Fonts/Deng.ttf"),
    Path("C:/Windows/Fonts/Dengb.ttf"),
]


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
    def __init__(self, initial_port: str = "", initial_baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.config = AppConfig()
        self.state = TelemetryState()
        self.initial_port = initial_port
        self.initial_baudrate = initial_baudrate
        self.service = ProtocolService()
        self.log_lines: Deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.plot_dirty = False
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
        }

    def run(self) -> None:
        dpg.create_context()
        self._setup_fonts()
        self._build_theme()
        self._build_ui()
        dpg.set_primary_window("main_window", True)
        self.refresh_ports(select_port=self.initial_port)
        dpg.set_value(self.tags["baud_input"], int(self.initial_baudrate))
        self._append_log("SYS", self.font_status)
        dpg.create_viewport(
            title="FOC Controller Host",
            width=1560,
            height=960,
            min_width=1260,
            min_height=820,
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
                with dpg.child_window(width=500, autosize_y=True, border=False):
                    self._build_connection_panel()
                    dpg.add_spacer(height=6)
                    self._build_command_panel()
                    dpg.add_spacer(height=6)
                    self._build_live_panel()
                with dpg.child_window(width=-1, autosize_y=True, border=False):
                    self._build_snapshot_panel()
                    dpg.add_spacer(height=6)
                    self._build_plot_panel()
                    dpg.add_spacer(height=6)
                    self._build_log_panel()

    def _build_connection_panel(self) -> None:
        with dpg.collapsing_header(label="串口连接", default_open=True):
            dpg.add_text("RS485 会话")
            with dpg.group(horizontal=True):
                dpg.add_combo(items=[], width=200, tag=self.tags["port_combo"], no_arrow_button=False)
                dpg.add_button(label="刷新串口", callback=lambda: self.refresh_ports())
            dpg.add_input_int(
                label="波特率",
                tag=self.tags["baud_input"],
                default_value=self.initial_baudrate,
                step=100000,
                min_value=9600,
                min_clamped=True,
                width=200,
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
                    width=120,
                    format="%.2f",
                )
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="自动校准查询", tag=self.tags["calib_poll_enabled"], default_value=False)
                dpg.add_input_float(
                    label="Hz ",
                    tag=self.tags["calib_poll_hz"],
                    default_value=DEFAULT_CALIB_POLL_HZ,
                    width=120,
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
                width=200,
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
                    width=120,
                )

            with dpg.tab_bar():
                with dpg.tab(label="控制"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.control_modes],
                        default_value=self.config.control_modes[0].label,
                        label="控制模式",
                        tag=self.tags["control_mode"],
                        width=240,
                    )
                    dpg.add_input_float(
                        label="目标值",
                        tag=self.tags["control_target"],
                        default_value=0.0,
                        width=240,
                        format="%.6f",
                    )
                    dpg.add_input_float(
                        label="前馈 0",
                        tag=self.tags["control_ff0"],
                        default_value=0.0,
                        width=240,
                        format="%.6f",
                    )
                    dpg.add_input_float(
                        label="前馈 1",
                        tag=self.tags["control_ff1"],
                        default_value=0.0,
                        width=240,
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
                        width=240,
                    )
                    dpg.add_button(label="读取选中参数", callback=lambda: self.send_param_query())

                with dpg.tab(label="参数写入"):
                    dpg.add_combo(
                        items=[item.label for item in self.config.param_defs],
                        default_value=self.config.param_defs[0].label,
                        label="参数项",
                        tag=self.tags["write_param_choice"],
                        width=240,
                    )
                    dpg.add_input_float(
                        label="参数值",
                        tag=self.tags["write_param_value"],
                        default_value=0.0,
                        width=240,
                        format="%.6f",
                    )
                    dpg.add_button(label="写入参数", callback=lambda: self.send_param_write())

                with dpg.tab(label="校准"):
                    dpg.add_text("收到 ACK 后固件会后台执行校准，进度请看校准状态回复。", wrap=420)
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
                        height=140,
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
                height=260,
                default_value="等待参数快照...",
            )

    def _build_plot_panel(self) -> None:
        with dpg.collapsing_header(label="实时曲线", default_open=True):
            self._create_plot("速度", "rad/s", self.tags["velocity_series"], "velocity_y_axis")
            self._create_plot("输出占空比", "duty", self.tags["output_series"], "output_y_axis")
            self._create_plot("位置", "rad", self.tags["position_series"], "position_y_axis")
            self._create_plot("母线电压", "V", self.tags["voltage_series"], "voltage_y_axis")

    def _build_log_panel(self) -> None:
        with dpg.collapsing_header(label="通信日志", default_open=True):
            dpg.add_input_text(
                tag=self.tags["log_text"],
                multiline=True,
                readonly=True,
                width=-1,
                height=300,
            )
            dpg.add_button(label="清空日志", callback=lambda: self.clear_logs())

    def _add_kv_row(self, key: str, value_tag: str, default_value: str) -> None:
        with dpg.table_row():
            dpg.add_text(key)
            dpg.add_text(default_value, tag=value_tag, wrap=320)

    def _create_plot(self, title: str, y_label: str, series_tag: str, y_axis_tag: str) -> None:
        with dpg.plot(label=title, height=170, width=-1):
            dpg.add_plot_legend()
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="样本")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=y_label, tag=y_axis_tag)
            dpg.add_line_series([], [], label=title, parent=y_axis, tag=series_tag)
            dpg.set_axis_limits_auto(x_axis)
            dpg.set_axis_limits_auto(y_axis)

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

        self.next_state_poll_at = time.monotonic()
        self.next_calib_poll_at = time.monotonic()
        dpg.set_value(self.tags["status_text"], f"连接中: {port} @ {baudrate}")
        self.service.connect(port=port, baudrate=baudrate)

    def disconnect(self) -> None:
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

        seq = self._next_seq()
        self.service.send_control(
            motor_id=self._selected_motor_id(),
            ctrl_mode=mode.value,
            target=float(dpg.get_value(self.tags["control_target"])),
            ff0=float(dpg.get_value(self.tags["control_ff0"])),
            ff1=float(dpg.get_value(self.tags["control_ff1"])),
            seq=seq,
        )

    def send_stop_command(self) -> None:
        seq = self._next_seq()
        self.service.send_control(
            motor_id=self._selected_motor_id(),
            ctrl_mode=CTRL_MODE_STOP,
            target=0.0,
            ff0=0.0,
            ff1=0.0,
            seq=seq,
        )

    def send_param_write(self) -> None:
        param_label = str(dpg.get_value(self.tags["write_param_choice"]))
        param = self._find_choice_by_label(self.config.param_defs, param_label)
        if param is None:
            self._append_log("ERR", f"未知参数项: {param_label}")
            return

        seq = self._next_seq()
        self.service.send_param_write(
            motor_id=self._selected_motor_id(),
            param_id=param.value,
            value=float(dpg.get_value(self.tags["write_param_value"])),
            seq=seq,
        )

    def send_state_query(self) -> None:
        self.service.query_state(
            motor_id=self._selected_motor_id(),
            seq=self._next_seq(),
        )

    def send_param_query(self) -> None:
        param_label = str(dpg.get_value(self.tags["query_param_choice"]))
        param = self._find_choice_by_label(self.config.param_defs, param_label)
        if param is None:
            self._append_log("ERR", f"未知参数项: {param_label}")
            return
        self.service.query_param(
            motor_id=self._selected_motor_id(),
            param_id=param.value,
            seq=self._next_seq(),
        )

    def send_snapshot_query(self) -> None:
        self.service.query_snapshot(
            motor_id=self._selected_motor_id(),
            seq=self._next_seq(),
        )

    def send_calibration_query(self) -> None:
        self.service.query_calibration_status(
            motor_id=self._selected_motor_id(),
            seq=self._next_seq(),
        )

    def send_calibration_start(self) -> None:
        self.service.start_calibration(
            motor_id=self._selected_motor_id(),
            seq=self._next_seq(),
        )

    def send_raw_command(self) -> None:
        self.service.send_raw_hex(str(dpg.get_value(self.tags["raw_hex"])))

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

        now = time.monotonic()

        if dpg.get_value(self.tags["poll_enabled"]):
            state_hz = max(0.2, float(dpg.get_value(self.tags["poll_hz"])))
            if now >= self.next_state_poll_at:
                self.send_state_query()
                self.next_state_poll_at = now + (1.0 / state_hz)

        calib_running = (
            self.state.last_calib is not None and self.state.last_calib.calib_state == CALIB_STATE_RUNNING
        )
        if dpg.get_value(self.tags["calib_poll_enabled"]) or calib_running:
            calib_hz = max(0.2, float(dpg.get_value(self.tags["calib_poll_hz"])))
            if now >= self.next_calib_poll_at:
                self.send_calibration_query()
                self.next_calib_poll_at = now + (1.0 / calib_hz)

    def _handle_connection_event(self, payload: object) -> None:
        info = payload if isinstance(payload, dict) else {}
        connected = bool(info.get("connected"))
        message = str(info.get("message", ""))

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
    parser.add_argument(
        "baudrate",
        nargs="?",
        default=DEFAULT_BAUDRATE,
        type=int,
        help="启动时默认波特率",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = FOCGuiApp(initial_port=args.port, initial_baudrate=args.baudrate)
    app.run()


if __name__ == "__main__":
    main()
