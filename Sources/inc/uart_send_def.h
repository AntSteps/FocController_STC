#ifndef __UART_SEND_DEF_H__
#define __UART_SEND_DEF_H__
#include "def.h"

#pragma pack(1)
typedef struct
{
    uint16_t head;   // 0x0721 代表上行
    uint8_t id;     // 电机 ID
    float velocity; // 当前速度
    float position; // 转子当前位置
    float current;  // 电机电流
    float voltage;  // 电机电压
    uint8_t status; // 0：就绪 0b1:编码器异常 0b10:电机异常 0b100: onanii wanted
    uint8_t seq;    // 时间戳
    uint8_t crc8;   // 校验位
} ReplyFrame; 
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint16_t head; // 0x1145 代表下行
    uint8_t id;   // 电机 ID
    uint8_t ctrl; // 控制模式 0代表休眠 1速度环 ...好像没有2了
    float target; // 控制目标值
    float ff[2];  // 可选的两个控制前馈项
    uint8_t seq;  // 时间戳
    uint8_t crc8; // 校验位
} CmdFrame;
#pragma pack()
#endif