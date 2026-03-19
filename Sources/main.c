
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
#include "command.h"
//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
// 在此添加用户全局变量定义、用户宏定义以及函数声明

static void LoadStoredParamsToModules(ParamStorage_Data_t *params);


void calibrate(void){
    ParamStorage_Data_t *params;
    Motor_Calibration new_calib;

    /* 从参数存储获取指针 */
    params = ParamStorage_Get();

    /* 执行自动校准，返回校准结果 */
    new_calib = Motor_Calibration_AutoCalibrate(params->motor_pole_pairs);

    /* 将新的校准结果应用到参数存储 */
    params->motor_calib = new_calib;
    ParamStorage_Save();                 /* 保存到EEPROM */

    /* 应用校准参数到运行模块 */
    Motor_Calibration_Init(&new_calib);
}


void main(void)
{
    UartCommand command;
    ParamStorage_Data_t *params;
    uint8_t reply_seq;

    SYS_Init();

    //<<AICUBE_USER_MAIN_CODE_BEGIN>>
    // 在此添加主函数中运行一次的用户代码
    Encoder_Init();
    ParamStorage_Init();                /* 初始化参数存储，从EEPROM加载参数 */

    /* 从参数存储获取指针 */
    params = ParamStorage_Get();

    TIMER0_Stop();

    /* 将存储参数或默认参数加载到各模块 */
    LoadStoredParamsToModules(params);

    SpeedController_SetTarget(20.0f);
    reply_seq = 0;

    TIMER0_Run();       /* 启动定时器 开始foc控制循环 */

    while (1)
    {
        while (UARTController_GetCommand(&command) == 0)
        {
            ProcessCommand(&command);
        }

        UartReply(reply_seq++);
        delay_ms(10);
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


static void LoadStoredParamsToModules(ParamStorage_Data_t *params)
{
    Motor_Calibration_Init(&params->motor_calib);
    SpeedController_Init(&params->speed_ctrl_params);
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


//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


