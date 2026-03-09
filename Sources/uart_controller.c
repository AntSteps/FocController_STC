#include "uart_controller.h"

CmdFrame cmdframe = {0};
ReplyFrame replyframe = {0};
ReplyFrame replyframe_to_send = {0}; // 储存大小端转换后的发送帧

uint32_t swap32(uint32_t x)
{
    return ((x & 0x000000FF) << 24) |
           ((x & 0x0000FF00) << 8)  |
           ((x & 0x00FF0000) >> 8)  |
           ((x & 0xFF000000) >> 24);
}

// 浮点型大小端转换函数（STC平台通用）
float swap_float(float x)
{
    union {
        float f;
        uint32_t u32;
    } tmp;
    tmp.f = x;
    tmp.u32 = swap32(tmp.u32);
    return tmp.f;
}

uint16_t swap16(uint16_t x)
{
    // 拆分高/低字节并交换
    uint8_t high_byte = (x >> 8) & 0xFF;  // 取高8位
    uint8_t low_byte  = x & 0xFF;         // 取低8位
    return (uint16_t)(low_byte << 8) | high_byte;  // 交换后合并
}

void send_messages(ReplyFrame *frame)
{
    replyframe_to_send.head = swap16(frame->head);
    replyframe_to_send.id = frame->id;
    replyframe_to_send.velocity = swap_float(frame->velocity);
    replyframe_to_send.position = swap_float(frame->position);
    replyframe_to_send.current = swap_float(frame->current);
    replyframe_to_send.voltage = swap_float(frame->voltage);
    replyframe_to_send.status = frame->status;
    replyframe_to_send.seq = frame->seq;
    replyframe_to_send.crc8 = Mem_CRC8_Check((uint8_t *)frame, sizeof(ReplyFrame) - 1, 0); // 计算CRC8校验码

    UART_DMA_Send((uint8_t *)&replyframe_to_send, sizeof(ReplyFrame));
}

// 返回0表示更新
uint8_t receive_messages(CmdFrame *frame)
{
    // 这里rxd_buf已经被DMA填充了接收到的数据
    if (read_ptr == write_ptr)
        return 1; // null_frame.

    while (read_ptr != write_ptr)
    {
        if (rxd_buf[read_ptr] == 0x11 && rxd_buf[(read_ptr + 1) % (18 * 5)] == 0x45) // 检测帧头0x1145
        {
            if ((write_ptr + 18 * 5 - read_ptr) % (18 * 5) >= sizeof(CmdFrame))
            {
                uint8_t temp_buf[sizeof(CmdFrame)];
                memcpy(temp_buf, &rxd_buf[read_ptr], sizeof(CmdFrame));

                if (!Mem_CRC8_Check(temp_buf, sizeof(CmdFrame), 1))
                {
                    *frame = *(CmdFrame *)temp_buf;
                    read_ptr = (read_ptr + sizeof(CmdFrame)) % (18 * 5);
                    return 0;
                }
                else
                {
                    read_ptr = (read_ptr + sizeof(CmdFrame)) % (18 * 5);
                    // debug here.
                    continue;
                }
            }
        }
        read_ptr = (read_ptr + 1) % (18 * 5);
    }
    return 1;
}