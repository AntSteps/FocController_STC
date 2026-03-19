#!/usr/bin/env python3
"""
FOC Controller 上位机 - 带实时波形图
需要安装: pip install pyserial matplotlib
"""

import serial
import struct
import sys
import time
import threading
from collections import deque
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import tkinter as tk
    from tkinter import ttk
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("警告: matplotlib 未安装，将使用命令行模式")
    print("安装: pip install matplotlib")

# 协议常量
REPLY_FRAME_HEAD = 0x0721
REPLY_FRAME_FORMAT = '>H B f f f f B B B'  # 大端模式
REPLY_FRAME_SIZE = struct.calcsize(REPLY_FRAME_FORMAT)


def crc8_check(data: bytes, has_crc: int = 0) -> int:
    """CRC8 校验"""
    crc = 0x00
    length = len(data) - has_crc
    for i in range(length):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


class FOCSerialReader:
    """串口数据读取器"""

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.buffer = bytearray()

        # 最新数据
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # 统计数据
        self.frame_count = 0
        self.error_count = 0
        self.last_seq = None
        self.lost_frames = 0

        # 历史数据
        self.history_len = 500
        self.time_history = deque(maxlen=self.history_len)
        self.velocity_history = deque(maxlen=self.history_len)
        self.output_history = deque(maxlen=self.history_len)
        self.position_history = deque(maxlen=self.history_len)
        self.voltage_history = deque(maxlen=self.history_len)
        self.start_time = time.time()

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )
            print(f"已连接到: {self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.serial:
            self.serial.close()

    def read_loop(self):
        """后台读取线程"""
        while self.running:
            if self.serial and self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                if data:
                    self.buffer.extend(data)
                    self.process_buffer()
            time.sleep(0.001)

    def process_buffer(self):
        """处理缓冲区数据"""
        while len(self.buffer) >= REPLY_FRAME_SIZE:
            # 查找帧头
            found = False
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == 0x07 and self.buffer[i + 1] == 0x21:
                    if i > 0:
                        self.buffer = self.buffer[i:]
                        self.error_count += 1
                    found = True
                    break

            if not found:
                self.buffer.clear()
                break

            if len(self.buffer) >= REPLY_FRAME_SIZE:
                frame_data = bytes(self.buffer[:REPLY_FRAME_SIZE])
                self.buffer = self.buffer[REPLY_FRAME_SIZE:]
                self.parse_frame(frame_data)
            else:
                break

    def parse_frame(self, data: bytes):
        """解析数据帧"""
        try:
            head, motor_id, velocity, output, position, voltage, status, seq, crc = \
                struct.unpack(REPLY_FRAME_FORMAT, data)

            if head != REPLY_FRAME_HEAD:
                return

            calc_crc = crc8_check(data, has_crc=1)
            if calc_crc != crc:
                self.error_count += 1
                return

            # 检查丢帧
            if self.last_seq is not None:
                expected = (self.last_seq + 1) & 0xFF
                if seq != expected:
                    self.lost_frames += (seq - expected) & 0xFF
            self.last_seq = seq

            self.frame_count += 1

            # 更新历史数据
            current_time = time.time() - self.start_time
            self.time_history.append(current_time)
            self.velocity_history.append(velocity)
            self.output_history.append(output)
            self.position_history.append(position)
            self.voltage_history.append(voltage)

            # 更新最新帧
            with self.frame_lock:
                self.latest_frame = {
                    'id': motor_id,
                    'velocity': velocity,
                    'output': output,
                    'position': position,
                    'voltage': voltage,
                    'status': status,
                    'seq': seq,
                    'time': current_time
                }
        except Exception as e:
            self.error_count += 1

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.daemon = True
        self.thread.start()

    def get_latest(self):
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame else None


class FOCMonitorGUI:
    """图形界面监控器"""

    def __init__(self, reader: FOCSerialReader):
        self.reader = reader
        self.root = tk.Tk()
        self.root.title("FOC Controller 监控器")
        self.root.geometry("1200x800")

        self.setup_ui()
        self.setup_plots()

        # 启动更新
        self.update_ui()
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plots, interval=50, cache_frame_data=False
        )

    def setup_ui(self):
        """设置用户界面"""
        # 顶部信息栏
        self.info_frame = ttk.Frame(self.root)
        self.info_frame.pack(fill='x', padx=10, pady=5)

        self.lbl_frame_count = ttk.Label(self.info_frame, text="帧数: 0")
        self.lbl_frame_count.pack(side='left', padx=10)

        self.lbl_error_count = ttk.Label(self.info_frame, text="错误: 0")
        self.lbl_error_count.pack(side='left', padx=10)

        self.lbl_lost_count = ttk.Label(self.info_frame, text="丢帧: 0")
        self.lbl_lost_count.pack(side='left', padx=10)

        self.lbl_voltage = ttk.Label(self.info_frame, text="电压: 0.00V", font=('Arial', 12, 'bold'))
        self.lbl_voltage.pack(side='right', padx=10)

        self.lbl_velocity = ttk.Label(self.info_frame, text="速度: 0.00 rad/s", font=('Arial', 12, 'bold'))
        self.lbl_velocity.pack(side='right', padx=10)

        # 按钮
        self.btn_frame = ttk.Frame(self.root)
        self.btn_frame.pack(fill='x', padx=10, pady=5)

        ttk.Button(self.btn_frame, text="保存数据", command=self.save_data).pack(side='left', padx=5)
        ttk.Button(self.btn_frame, text="清除显示", command=self.clear_plots).pack(side='left', padx=5)
        ttk.Button(self.btn_frame, text="退出", command=self.on_close).pack(side='right', padx=5)

    def setup_plots(self):
        """设置波形图"""
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.tight_layout(pad=3.0)

        # 速度图
        self.ax_velocity = self.axes[0, 0]
        self.ax_velocity.set_title('速度 (rad/s)')
        self.ax_velocity.set_xlabel('时间 (s)')
        self.ax_velocity.set_ylabel('速度')
        self.ax_velocity.grid(True)
        self.line_velocity, = self.ax_velocity.plot([], [], 'b-', linewidth=1)

        # 输出图
        self.ax_output = self.axes[0, 1]
        self.ax_output.set_title('输出占空比')
        self.ax_output.set_xlabel('时间 (s)')
        self.ax_output.set_ylabel('占空比')
        self.ax_output.grid(True)
        self.line_output, = self.ax_output.plot([], [], 'g-', linewidth=1)

        # 位置图
        self.ax_position = self.axes[1, 0]
        self.ax_position.set_title('位置 (rad)')
        self.ax_position.set_xlabel('时间 (s)')
        self.ax_position.set_ylabel('位置')
        self.ax_position.grid(True)
        self.line_position, = self.ax_position.plot([], [], 'r-', linewidth=1)

        # 电压图
        self.ax_voltage = self.axes[1, 1]
        self.ax_voltage.set_title('母线电压 (V)')
        self.ax_voltage.set_xlabel('时间 (s)')
        self.ax_voltage.set_ylabel('电压')
        self.ax_voltage.grid(True)
        self.line_voltage, = self.ax_voltage.plot([], [], 'm-', linewidth=1)

        # 嵌入到 Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=5)

    def update_plots(self, frame):
        """更新波形图"""
        if len(self.reader.time_history) < 2:
            return

        t = list(self.reader.time_history)

        # 更新速度图
        self.line_velocity.set_data(t, list(self.reader.velocity_history))
        self.ax_velocity.set_xlim(t[0], t[-1])
        if len(self.reader.velocity_history) > 0:
            v_min, v_max = min(self.reader.velocity_history), max(self.reader.velocity_history)
            margin = max(0.1, (v_max - v_min) * 0.1)
            self.ax_velocity.set_ylim(v_min - margin, v_max + margin)

        # 更新输出图
        self.line_output.set_data(t, list(self.reader.output_history))
        self.ax_output.set_xlim(t[0], t[-1])
        if len(self.reader.output_history) > 0:
            o_min, o_max = min(self.reader.output_history), max(self.reader.output_history)
            margin = max(0.01, (o_max - o_min) * 0.1)
            self.ax_output.set_ylim(o_min - margin, o_max + margin)

        # 更新位置图
        self.line_position.set_data(t, list(self.reader.position_history))
        self.ax_position.set_xlim(t[0], t[-1])
        if len(self.reader.position_history) > 0:
            p_min, p_max = min(self.reader.position_history), max(self.reader.position_history)
            margin = max(0.1, (p_max - p_min) * 0.1)
            self.ax_position.set_ylim(p_min - margin, p_max + margin)

        # 更新电压图
        self.line_voltage.set_data(t, list(self.reader.voltage_history))
        self.ax_voltage.set_xlim(t[0], t[-1])
        if len(self.reader.voltage_history) > 0:
            v_min, v_max = min(self.reader.voltage_history), max(self.reader.voltage_history)
            margin = max(0.1, (v_max - v_min) * 0.1)
            self.ax_voltage.set_ylim(v_min - margin, v_max + margin)

        return self.line_velocity, self.line_output, self.line_position, self.line_voltage

    def update_ui(self):
        """更新 UI 信息"""
        self.lbl_frame_count.config(text=f"帧数: {self.reader.frame_count}")
        self.lbl_error_count.config(text=f"错误: {self.reader.error_count}")
        self.lbl_lost_count.config(text=f"丢帧: {self.reader.lost_frames}")

        latest = self.reader.get_latest()
        if latest:
            self.lbl_velocity.config(text=f"速度: {latest['velocity']:.2f} rad/s")
            self.lbl_voltage.config(text=f"电压: {latest['voltage']:.2f}V")

        self.root.after(100, self.update_ui)

    def save_data(self):
        """保存数据到 CSV"""
        filename = f"foc_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w') as f:
            f.write("time,velocity,output,position,voltage\n")
            for i in range(len(self.reader.time_history)):
                f.write(f"{self.reader.time_history[i]},"
                       f"{self.reader.velocity_history[i]},"
                       f"{self.reader.output_history[i]},"
                       f"{self.reader.position_history[i]},"
                       f"{self.reader.voltage_history[i]}\n")
        print(f"数据已保存到: {filename}")

    def clear_plots(self):
        """清除显示"""
        self.reader.time_history.clear()
        self.reader.velocity_history.clear()
        self.reader.output_history.clear()
        self.reader.position_history.clear()
        self.reader.voltage_history.clear()

    def on_close(self):
        self.reader.disconnect()
        self.root.destroy()

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()


def main():
    if not MATPLOTLIB_AVAILABLE:
        print("请先安装 matplotlib: pip install matplotlib")
        return

    # 串口参数
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

    # 创建读取器
    reader = FOCSerialReader(port, baudrate)
    if not reader.connect():
        return

    reader.start()

    # 创建 GUI
    gui = FOCMonitorGUI(reader)
    gui.run()


if __name__ == '__main__':
    main()
