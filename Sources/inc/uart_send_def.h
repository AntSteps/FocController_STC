#ifndef __UART_SEND_DEF_H__
#define __UART_SEND_DEF_H__
#include "def.h"

#define REPLY_FRAME_HEAD            0x0721
#define CTRL_CMD_FRAME_HEAD         0x1145
#define PARAM_CMD_FRAME_HEAD        0x1245

#define CTRL_CMD_HEAD_BYTE0         0x11
#define CTRL_CMD_HEAD_BYTE1         0x45
#define PARAM_CMD_HEAD_BYTE0        0x12
#define PARAM_CMD_HEAD_BYTE1        0x45

#define PARAM_CMD_ID_VEL_KP         0x01
#define PARAM_CMD_ID_VEL_KI         0x02
#define PARAM_CMD_ID_VEL_LIMIT      0x03
#define PARAM_CMD_ID_CUR_LIMIT      0x04
#define PARAM_CMD_ID_FF_GAIN        0x05
#define PARAM_CMD_ID_FF_ENABLE      0x06

typedef enum
{
    UART_COMMAND_NONE = 0,
    UART_COMMAND_CTRL = 1,
    UART_COMMAND_PARAM = 2
} UartCommandType;

#pragma pack(1)
typedef struct
{
    uint16_t head;   // 0x0721 代表上行
    uint8_t id;     // 电机 ID
    float velocity; // 当前速度
    float output;   // 当前控制输出（占空比）
    float position; // 转子当前位置
    float voltage;  // 电机电压
    uint8_t status; // 0：就绪 0b1:编码器异常 0b10:电机异常
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
} CtrlCmdFrame;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint16_t head;   // 0x1245 代表参数下发
    uint8_t id;      // 电机 ID
    uint8_t param;   // 参数 ID
    float value;     // 参数值
    uint8_t seq;     // 时间戳
    uint8_t crc8;    // 校验位
} ParamCmdFrame;
#pragma pack()

typedef struct
{
    UartCommandType type;
    CtrlCmdFrame ctrl;
    ParamCmdFrame param;
} UartCommand;
#endif