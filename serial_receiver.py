#!/usr/bin/env python3
"""
串口数据接收器 - 接收电机参数辨识数据帧
帧格式: 0xAA 0x55 + q_axis(4字节float) + elec_angle(4字节float) + 0x55 0xAA
"""

import serial
import struct
import csv
import sys
import argparse
from datetime import datetime


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='接收电机辨识数据并通过串口保存为CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python serial_receiver.py COM3 115200
  python serial_receiver.py /dev/ttyUSB0 921600 -o data.csv
        '''
    )
    parser.add_argument('port', help='串口号 (如 COM3 或 /dev/ttyUSB0)')
    parser.add_argument('baudrate', type=int, help='波特率 (如 115200, 921600)')
    parser.add_argument('-o', '--output', default=None,
                        help='输出CSV文件名 (默认: YYYYMMDD_HHMMSS.csv)')
    parser.add_argument('-t', '--timeout', type=float, default=1.0,
                        help='串口超时时间(秒), 默认1.0')
    parser.add_argument('--no-save', action='store_true',
                        help='不保存文件, 仅打印到屏幕')
    return parser.parse_args()


def receive_data(ser, output_file=None):
    """
    接收串口数据并解析帧
    状态机: WAIT_HEAD1 -> WAIT_HEAD2 -> READ_DATA1(4字节) -> READ_DATA2(4字节) -> WAIT_TAIL1 -> WAIT_TAIL2
    """
    STATE_WAIT_HEAD1 = 0
    STATE_WAIT_HEAD2 = 1
    STATE_READ_DATA = 2
    STATE_WAIT_TAIL1 = 3
    STATE_WAIT_TAIL2 = 4

    state = STATE_WAIT_HEAD1
    data_buffer = bytearray()
    float_data = bytearray(8)  # 存储两个float的原始字节
    data_idx = 0

    samples = []
    sample_count = 0
    byte_count = 0

    print(f"开始接收数据... 按 Ctrl+C 停止")
    print(f"帧格式: [AA 55] [q_axis:4B] [elec_angle:4B] [55 AA]")
    print("-" * 50)

    try:
        while True:
            # 读取一个字节
            byte = ser.read(1)
            if not byte:
                continue

            byte_val = byte[0]
            byte_count += 1

            if state == STATE_WAIT_HEAD1:
                if byte_val == 0xAA:
                    state = STATE_WAIT_HEAD2

            elif state == STATE_WAIT_HEAD2:
                if byte_val == 0x55:
                    state = STATE_READ_DATA
                    data_idx = 0
                    float_data = bytearray(8)
                else:
                    state = STATE_WAIT_HEAD1
                    if byte_val == 0xAA:  # 可能是新的帧头开始
                        state = STATE_WAIT_HEAD2

            elif state == STATE_READ_DATA:
                float_data[data_idx] = byte_val
                data_idx += 1
                if data_idx >= 8:
                    state = STATE_WAIT_TAIL1

            elif state == STATE_WAIT_TAIL1:
                if byte_val == 0x55:
                    state = STATE_WAIT_TAIL2
                else:
                    # 帧尾不匹配, 重置
                    state = STATE_WAIT_HEAD1
                    if byte_val == 0xAA:
                        state = STATE_WAIT_HEAD2

            elif state == STATE_WAIT_TAIL2:
                if byte_val == 0xAA:
                    # 完整帧接收成功, 解析数据 (大端模式 >f)
                    try:
                        q_axis = struct.unpack('>f', float_data[0:4])[0]
                        elec_angle = struct.unpack('>f', float_data[4:8])[0]

                        sample_count += 1
                        samples.append({
                            'sample': sample_count,
                            'q_axis': q_axis,
                            'elec_angle': elec_angle
                        })

                        # 打印到屏幕 (每100个采样打印一次, 避免刷屏)
                        if sample_count % 100 == 0:
                            print(f"已接收 {sample_count} 个样本, "
                                  f"最新: q={q_axis:.4f}, angle={elec_angle:.4f}")
                        elif sample_count <= 10 or sample_count % 10 == 0:
                            print(f"[{sample_count:4d}] q_axis={q_axis:.6f}, "
                                  f"elec_angle={elec_angle:.6f}")

                    except struct.error as e:
                        print(f"解析错误: {e}")

                    state = STATE_WAIT_HEAD1
                else:
                    state = STATE_WAIT_HEAD1
                    if byte_val == 0xAA:
                        state = STATE_WAIT_HEAD2

    except KeyboardInterrupt:
        print(f"\n{'='*50}")
        print(f"用户停止接收")
        print(f"总接收字节数: {byte_count}")
        print(f"有效样本数: {sample_count}")

    return samples


def save_to_csv(samples, filename):
    """保存数据到CSV文件"""
    if not samples:
        print("没有数据需要保存")
        return

    with open(filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['sample', 'q_axis', 'elec_angle'])
        writer.writeheader()
        writer.writerows(samples)

    print(f"数据已保存到: {filename}")
    print(f"样本数: {len(samples)}")


def main():
    args = parse_args()

    # 生成默认文件名
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"motor_data_{timestamp}.csv"

    # 打开串口
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout
        )
        print(f"串口已打开: {args.port} @ {args.baudrate}bps")
    except serial.SerialException as e:
        print(f"无法打开串口 {args.port}: {e}")
        sys.exit(1)

    # 接收数据
    try:
        samples = receive_data(ser, args.output)
    finally:
        ser.close()
        print(f"串口已关闭")

    # 保存数据
    if not args.no_save and samples:
        save_to_csv(samples, args.output)


if __name__ == '__main__':
    main()
