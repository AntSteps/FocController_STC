#include "uart_controller.h"

CmdFrame cmdframe = {0};
ReplyFrame replyframe = {0};
ReplyFrame replyframe_to_send = {0}; // 储存大小端转换后的发送帧

uint16_t buf_ptr = 0; // 接收缓冲区指针

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

// 接收并解析数据帧，返回0表示成功，1表示失败（如CRC校验失败或未找到帧头）
uint8_t receive_messages(CmdFrame *frame)
{
    // 这里假设rxd_buf已经被DMA填充了接收到的数据 (应该在DMA接收中断里处理数据并调用这个函数)
    // TODO: 实际使用时需要根据具体协议实现帧头检测、数据提取和CRC校验等逻辑
    return 1;
}