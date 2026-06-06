#include "config.h" //默认已包含stdio.h、intrins.h等头文件
#include "encoder.h"
#include "pwm.h"
#include "svpwm.h"
#include "param_storage.h"
#include "param_registry.h"
#include "uart_controller.h"
#include "command.h"
#include "calibration_service.h"
#include "motor.h"
#include "debug.h"

void main(void)
{
#if !DEBUG_MODE_ENABLE
    ProtocolRequest request;
#endif

    SYS_Init();
    TIMER0_Stop();

    ParamStorage_Init();
    ParamRegistry_ApplyAllRuntime();

#if DEBUG_MODE_ENABLE
    Motor_SetControlMode(MOTOR_CONTROL_MODE_SPEED);
    TIMER0_Run();
    Debug_Init();
    while (1)
    {
        Debug_Poll();
    }
#else
    ProtocolController_Init();
    CalibrationService_Init();
    Motor_SetControlMode(MOTOR_CONTROL_MODE_STOP);
    TIMER0_Run();
    while (1)
    {
        if (ProtocolController_PumpRx(&request))
        {
            MOTOR_STATUS=!MOTOR_STATUS;
            ProcessCommand(&request);
            MOTOR_STATUS=!MOTOR_STATUS;
        }
        CalibrationService_Poll();
    }
#endif
}

////////////////////////////////////////
// 系统初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SYS_Init(void)
{
    EnableAccessXFR();    // 使能访问扩展XFR
    AccessCodeFastest();  // 设置最快速度访问程序代码
    AccessIXramFastest(); // 设置最快速度访问内部XDATA
    IAP_SetTimeBase();    // 设置IAP等待参数,产生1us时基

    //<<AICUBE_USER_PREINITIAL_CODE_BEGIN>>
    // 在此添加用户预初始化代码
    //<<AICUBE_USER_PREINITIAL_CODE_END>>

    PORT0_Init();  // P0口初始化
    PORT1_Init();  // P1口初始化
    PORT2_Init();  // P2口初始化
    PORT3_Init();  // P3口初始化
    PORT5_Init();  // P5口初始化
    CLK_Init();    // 时钟模块初始化
    TIMER0_Init(); // 定时器0初始化
    TIMER3_Init(); // 定时器3初始化
    UART3_Init();  // 串口3初始化
    ADC_Init();    // ADC初始化
    SPI_Init();    // SPI初始化
    PWMA_Init();   /* 高级PWMA初始化 */
    // Motor_PowerOn_SelfTest();
    MATHLIB_Init(); /* MATH库初始化 */
    SVPWM_Init();   /* SVPWM初始化 */
    IAP_Init();

    //<<AICUBE_USER_INITIAL_CODE_BEGIN>>
    // 在此添加用户初始化代码
    Encoder_Init();

    //<<AICUBE_USER_INITIAL_CODE_END>>

    EnableGlobalInt(); // 使能全局中断
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
        NOP(34); //(MAIN_Fosc + 500000) / 1000000 - 6
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
        while (--i)
            ;
    } while (--ms);
}
