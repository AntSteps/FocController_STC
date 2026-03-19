//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息
// 文件名称: main.c
// 文件描述:
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-06) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#include "config.h"                     //默认已包含stdio.h、intrins.h等头文件


//<<AICUBE_USER_INCLUDE_BEGIN>>
// 在此添加用户头文件包含
#include "encoder.h"
#include "pwm.h"
#include "svpwm.h"
#include "calibration.h"
#include "param_storage.h"
#include "control.h"
#include "uart_send_def.h"
#include "uart_controller.h"
//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
// 在此添加用户全局变量定义、用户宏定义以及函数声明

void ReplyFrame_Assemble(void);
static void ProcessCommand(UartCommand *command);
static void ApplyParamCommand(ParamCmdFrame *frame);

//<<AICUBE_USER_GLOBAL_DEFINE_END>>



////////////////////////////////////////
// 项目主函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void main(void)
{
    UartCommand command;
    ParamStorage_Data_t *params;
    Motor_Calibration new_calib;
    /* 速度环参数结构体，静态存储保证生命周期，C89需在声明时初始化 */
    static SpeedController_Params_t speed_ctrl_params = {
        0.001f,         /* kp */
        0.000f,         /* ki */
        1.0f,           /* out_max */
        -1.0f,          /* out_min */
        5.0f,           /* lpf_time_constant_ms */
        {0.0f, 0.0f, 0.0f}, /* ff: velocity, acceleration, friction */
        0,              /* enable_feedforward */
        300.0f          /* sample_freq_hz */
    };

    SYS_Init();

    //<<AICUBE_USER_MAIN_CODE_BEGIN>>
    // 在此添加主函数中运行一次的用户代码
    Encoder_Init();
    ParamStorage_Init();                /* 初始化参数存储，从EEPROM加载参数 */

    /* 从参数存储获取指针 */
    params = ParamStorage_Get();
    speed_ctrl_params.kp = params->vel_kp;
    speed_ctrl_params.ki = params->vel_ki;

    TIMER0_Stop();

    /* 用参数存储中的校准数据初始化（如果EEPROM中有有效数据） */
    Motor_Calibration_Init(&params->motor_calib);

    /* 执行自动校准，返回校准结果 */
    new_calib = Motor_Calibration_AutoCalibrate(params->motor_pole_pairs);

    /* 将新的校准结果应用到参数存储 */
    params->motor_calib = new_calib;
    ParamStorage_Save();                 /* 保存到EEPROM */

    /* 应用校准参数到运行模块 */
    Motor_Calibration_Apply(&new_calib);

    /* 初始化速度环 */
    SpeedController_Init(&speed_ctrl_params);

    /* 设置目标速度 (rad/s), 目标100 rad/s (约955 rpm) */
    SpeedController_SetTarget(20.0f);

    TIMER0_Run();       /* 定时器接管闭环控制 */
    // printf("Hello World !\n");
    /*<<AICUBE_USER_MAIN_CODE_END>>*/

    while (1)
    {
        /*<AICUBE_USER_MAIN_LOOP_BEGIN>*/
        // while (UARTController_GetCommand(&command) == 0)
        // {
        //     ProcessCommand(&command);
        // }
        ReplyFrame_Assemble();
        delay_ms(10);  /* 100Hz 发送频率 */
        //<<AICUBE_USER_MAIN_LOOP_END>>
    }
}

////////////////////////////////////////
// 系统初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SYS_Init(void)
{
    EnableAccessXFR();                  //使能访问扩展XFR
    AccessCodeFastest();                //设置最快速度访问程序代码
    AccessIXramFastest();               //设置最快速度访问内部XDATA
    IAP_SetTimeBase();                  //设置IAP等待参数,产生1us时基

    //<<AICUBE_USER_PREINITIAL_CODE_BEGIN>>
    // 在此添加用户预初始化代码
    //<<AICUBE_USER_PREINITIAL_CODE_END>>

    PORT0_Init();   // P0口初始化
    PORT1_Init();   // P1口初始化
    PORT2_Init();   // P2口初始化
    PORT3_Init();   // P3口初始化
    PORT5_Init();   // P5口初始化
    CLK_Init();     // 时钟模块初始化
    TIMER0_Init();  // 定时器0初始化
    TIMER3_Init();  // 定时器3初始化
    UART3_Init();   // 串口3初始化
    ADC_Init();     // ADC初始化
    SPI_Init();     // SPI初始化
    PWMA_Init();    /* 高级PWMA初始化 */
    MATHLIB_Init(); /* MATH库初始化 */
    SVPWM_Init();   /* SVPWM初始化 */
    IAP_Init();

    //<<AICUBE_USER_INITIAL_CODE_BEGIN>>
    // 在此添加用户初始化代码
    Encoder_Init();

    //<<AICUBE_USER_INITIAL_CODE_END>>

    EnableGlobalInt();                  //使能全局中断
}

////////////////////////////////////////
// 微秒延时函数
// 入口参数: us (设置延时的微秒值)
// 函数返回: 无
////////////////////////////////////////
void delay_us(uint16_t us)
{
    do
    {
        NOP(34);                        //(MAIN_Fosc + 500000) / 1000000 - 6
    } while (--us);
}


////////////////////////////////////////
// 毫秒延时函数
// 入口参数: ms (设置延时的毫秒值)
// 函数返回: 无
////////////////////////////////////////
void delay_ms(uint16_t ms)
{
    uint16_t i;

    do
    {
        i = MAIN_Fosc / 6000;
        while (--i);
    } while (--ms);
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码

////////////////////////////////////////
// 组装并发送ReplyFrame
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void ReplyFrame_Assemble(void)
{
    /* 填充帧头 (大端: 0x07 0x21) */
    replyframe.head = REPLY_FRAME_HEAD;

    /* 电机ID */
    replyframe.id = 0x01;

    /* TODO: 用户在这里填充实际数据，目前使用dummy数据 */
    replyframe.velocity = 3.14159f;     /* 当前速度 rad/s */
    replyframe.output = 0.5f;           /* 控制输出/占空比 */
    replyframe.position = 1.234f;       /* 转子位置 rad */
    replyframe.voltage = ADC_GetBusVoltage();  /* 母线电压 V */

    /* 状态: 0=就绪, bit0=编码器异常, bit1=电机异常 */
    replyframe.status = 0x00;

    /* 时间戳递增 */
    replyframe.seq++;

    /* 发送数据帧 */
    send_messages(&replyframe);
}

static void ProcessCommand(UartCommand *command)
{
    ParamStorage_Data_t *params;
    float target;

    params = ParamStorage_Get();
    if (command->type == UART_COMMAND_CTRL)
    {
        if (command->ctrl.ctrl == 0)
        {
            SpeedController_SetTarget(0.0f);
            return;
        }

        if (command->ctrl.ctrl == 1)
        {
            target = command->ctrl.target;
            if (target > params->velocity_limit)
            {
                target = params->velocity_limit;
            }
            else if (target < -params->velocity_limit)
            {
                target = -params->velocity_limit;
            }
            SpeedController_SetTarget(target);
        }
        return;
    }

    if (command->type == UART_COMMAND_PARAM)
    {
        ApplyParamCommand(&command->param);
    }
}

static void ApplyParamCommand(ParamCmdFrame *frame)
{
    ParamStorage_Data_t *params;

    params = ParamStorage_Get();
    switch (frame->param)
    {
    case PARAM_CMD_ID_VEL_KP:
        params->vel_kp = frame->value;
        if (g_speed_ctrl.params != NULL)
        {
            g_speed_ctrl.params->kp = frame->value;
            PI_Init(&g_speed_ctrl.pi,
                    g_speed_ctrl.params->kp,
                    g_speed_ctrl.params->ki,
                    g_speed_ctrl.params->out_max,
                    g_speed_ctrl.params->out_min);
        }
        break;

    case PARAM_CMD_ID_VEL_KI:
        params->vel_ki = frame->value;
        if (g_speed_ctrl.params != NULL)
        {
            g_speed_ctrl.params->ki = frame->value;
            PI_Init(&g_speed_ctrl.pi,
                    g_speed_ctrl.params->kp,
                    g_speed_ctrl.params->ki,
                    g_speed_ctrl.params->out_max,
                    g_speed_ctrl.params->out_min);
        }
        break;

    case PARAM_CMD_ID_VEL_LIMIT:
        params->velocity_limit = frame->value;
        break;

    case PARAM_CMD_ID_CUR_LIMIT:
        params->current_limit = frame->value;
        break;

    case PARAM_CMD_ID_FF_GAIN:
        if (g_speed_ctrl.params != NULL)
        {
            g_speed_ctrl.params->ff.velocity_gain = frame->value;
        }
        break;

    case PARAM_CMD_ID_FF_ENABLE:
        if (g_speed_ctrl.params != NULL)
        {
            g_speed_ctrl.params->enable_feedforward = (uint8_t)(frame->value != 0.0f);
        }
        break;

    default:
        return;
    }

    ParamStorage_Save();
}

//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


