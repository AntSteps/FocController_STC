#!/usr/bin/env python3
"""
FOC Controller 上位机
接收 ReplyFrame 数据并实时展示
"""

import serial
import struct
import sys
import time
from collections import deque
import threading

# 协议常量
REPLY_FRAME_HEAD = 0x0721
CMD_FRAME_HEAD = 0x1145

# ReplyFrame 结构体定义 (大端模式)
# head: uint16_t, id: uint8_t, velocity: float, output: float,
# position: float, voltage: float, status: uint8_t, seq: uint8_t, crc8: uint8_t
REPLY_FRAME_FORMAT = '>H B f f f f B B B'  # 大端: H=uint16, B=uint8, f=float
REPLY_FRAME_SIZE = struct.calcsize(REPLY_FRAME_FORMAT)

# CRC8 校验表 (与 STC 端一致，使用标准 CRC-8)
def crc8_check(data: bytes, has_crc: int = 0) -> int:
    """
    计算 CRC8 校验值
    data: 字节数据
    has_crc: 是否包含 CRC 字节（用于校验时）
    返回: CRC8 值
    """
    crc = 0x00
    length = len(data) - has_crc

    for i in range(length):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31  # 标准 CRC-8 多项式 x^8 + x^5 + x^4 + 1
            else:
                crc <<= 1
            crc &= 0xFF

    return crc


def parse_reply_frame(data: bytes) -> dict:
    """解析 ReplyFrame 数据包"""
    if len(data) != REPLY_FRAME_SIZE:
        return None

    # 解包
    head, motor_id, velocity, output, position, voltage, status, seq, crc = \
        struct.unpack(REPLY_FRAME_FORMAT, data)

    # 校验帧头
    if head != REPLY_FRAME_HEAD:
        return None

    # 校验 CRC
    calc_crc = crc8_check(data, has_crc=1)
    if calc_crc != crc:
        print(f"CRC 校验失败: 计算值={calc_crc:02X}, 接收值={crc:02X}")
        return None

    return {
        'head': hex(head),
        'id': motor_id,
        'velocity': velocity,
        'output': output,
        'position': position,
        'voltage': voltage,
        'status': status,
        'status_str': parse_status(status),
        'seq': seq,
        'crc': crc
    }


def parse_status(status: int) -> str:
    """解析状态字节"""
    if status == 0:
        return "就绪"
    flags = []
    if status & 0x01:
        flags.append("编码器异常")
    if status & 0x02:
        flags.append("电机异常")
    return ",".join(flags) if flags else "未知状态"


class FOCMonitor:
    """FOC 控制器监控器"""

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.buffer = bytearray()

        # 数据统计
        self.frame_count = 0
        self.error_count = 0
        self.last_seq = None
        self.lost_frames = 0

        # 历史数据（用于绘图）
        self.history_len = 500
        self.velocity_history = deque(maxlen=self.history_len)
        self.output_history = deque(maxlen=self.history_len)
        self.position_history = deque(maxlen=self.history_len)
        self.voltage_history = deque(maxlen=self.history_len)

    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"已连接到串口: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口"""
        self.running = False
        if self.serial:
            self.serial.close()
            print("串口已关闭")

    def find_frame_header(self) -> int:
        """在缓冲区中查找帧头位置"""
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == 0x07 and self.buffer[i + 1] == 0x21:
                return i
        return -1

    def process_data(self):
        """处理接收到的数据"""
        while self.running and self.serial:
            # 读取数据
            data = self.serial.read(64)
            if data:
                self.buffer.extend(data)

            # 处理缓冲区中的完整帧
            while len(self.buffer) >= REPLY_FRAME_SIZE:
                header_pos = self.find_frame_header()

                if header_pos < 0:
                    # 没有找到帧头，清空缓冲区
                    self.buffer.clear()
                    break

                if header_pos > 0:
                    # 丢弃帧头前的数据
                    self.buffer = self.buffer[header_pos:]
                    self.error_count += 1

                if len(self.buffer) >= REPLY_FRAME_SIZE:
                    # 提取完整帧
                    frame_data = bytes(self.buffer[:REPLY_FRAME_SIZE])
                    self.buffer = self.buffer[REPLY_FRAME_SIZE:]

                    # 解析帧
                    frame = parse_reply_frame(frame_data)
                    if frame:
                        self.handle_frame(frame)
                    else:
                        self.error_count += 1
                        # 丢弃一个字节继续查找
                        self.buffer = bytearray(frame_data[1:]) + self.buffer

    def handle_frame(self, frame: dict):
        """处理解析后的数据帧"""
        self.frame_count += 1

        # 检查丢帧
        if self.last_seq is not None:
            expected_seq = (self.last_seq + 1) & 0xFF
            if frame['seq'] != expected_seq:
                lost = (frame['seq'] - expected_seq) & 0xFF
                self.lost_frames += lost
        self.last_seq = frame['seq']

        # 添加到历史数据
        self.velocity_history.append(frame['velocity'])
        self.output_history.append(frame['output'])
        self.position_history.append(frame['position'])
        self.voltage_history.append(frame['voltage'])

        # 显示数据
        self.display_frame(frame)

    def display_frame(self, frame: dict):
        """显示数据帧信息"""
        # 清屏并重新显示
        print("\033[2J\033[H", end="")  # ANSI 清屏
        print("=" * 70)
        print("FOC Controller 监控器")
        print("=" * 70)
        print(f"帧计数: {self.frame_count} | 错误计数: {self.error_count} | 丢帧: {self.lost_frames}")
        print("-" * 70)
        print(f"电机 ID:     {frame['id']}")
        print(f"序列号:      {frame['seq']}")
        print(f"状态:        {frame['status_str']} (0x{frame['status']:02X})")
        print("-" * 70)
        print(f"速度:        {frame['velocity']:10.3f} rad/s")
        print(f"输出占空比:  {frame['output']:10.3f}")
        print(f"位置:        {frame['position']:10.3f} rad")
        print(f"母线电压:    {frame['voltage']:10.3f} V")
        print("=" * 70)

        # 显示统计数据
        if len(self.velocity_history) > 0:
            print("\n统计信息:")
            print(f"  速度 - 均值: {sum(self.velocity_history)/len(self.velocity_history):.3f}, "
                  f"最大: {max(self.velocity_history):.3f}, 最小: {min(self.velocity_history):.3f}")
            print(f"  电压 - 均值: {sum(self.voltage_history)/len(self.voltage_history):.3f} V")

    def run(self):
        """启动监控"""
        if not self.connect():
            return

        self.running = True
        print("\n按 Ctrl+C 停止...\n")

        try:
            self.process_data()
        except KeyboardInterrupt:
            print("\n用户停止")
        finally:
            self.disconnect()
            self.print_summary()

    def print_summary(self):
        """打印统计摘要"""
        print("\n" + "=" * 70)
        print("运行摘要")
        print("=" * 70)
        print(f"总接收帧数: {self.frame_count}")
        print(f"错误帧数:   {self.error_count}")
        print(f"丢帧数:     {self.lost_frames}")
        if self.frame_count > 0:
            print(f"错误率:     {100 * self.error_count / self.frame_count:.2f}%")
            print(f"丢帧率:     {100 * self.lost_frames / (self.frame_count + self.lost_frames):.2f}%")


def list_serial_ports():
    """列出可用串口"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用串口")
        return []
    print("可用串口:")
    for i, port in enumerate(ports):
        print(f"  {i + 1}. {port.device} - {port.description}")
    return [p.device for p in ports]


def main():
    """主函数"""
    # 解析命令行参数
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        ports = list_serial_ports()
        if not ports:
            print("请指定串口: python host_pc.py COM3")
            sys.exit(1)
        port = ports[0]

    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

    # 创建并运行监控器
    monitor = FOCMonitor(port, baudrate)
    monitor.run()


if __name__ == '__main__':
    main()
