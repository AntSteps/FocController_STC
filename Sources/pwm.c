//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: pwm.c
// 文件描述: 
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-06) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#include "config.h"


//<<AICUBE_USER_INCLUDE_BEGIN>>
// 在此添加用户头文件包含  
//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
// 在此添加用户全局变量定义、用户宏定义以及函数声明  
//<<AICUBE_USER_GLOBAL_DEFINE_END>>



////////////////////////////////////////
// PWMA初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void PWMA_Init(void)
{
    PWMA_C1SwitchP0001();               //设置PWMA通道1数据端口: PWM1P (P0.0), PWM1N (P0.1)
    PWMA_C2SwitchP0203();               //设置PWMA通道2数据端口: PWM2P (P0.2), PWM2N (P0.3)
    PWMA_C3SwitchP1415();               //设置PWMA通道3数据端口: PWM3P (P1.4), PWM3N (P1.5)
    PWMA_C4SwitchP1617();               //设置PWMA通道4数据端口: PWM4P (P1.6), PWM4N (P1.7)

    HSPWMA_Disable();                   //关闭高速模式，使用同步方式进行初始化
    PWMA_InternalClockMode();           //选择内部时钟模式
    PWMA_SetClockDivider(1);            //设置16位预分频
    PWMA_SetReload16(399);              //设置16位重载值
    PWMA_UnbufferARR();                 //关闭重载值寄存器预装载功能
    PWMA_SetCounter(0);                 //初始化计数值
    PWMA_SetDeadTime(0);                //设置死区时间为：无死区
    PWMA_CenterAlignMode2();            //设置计数器计数为中央对齐模式
    PWMA_ContinueMode();                //设置计数器为连续计数模式
    PWMA_SetRepeatCounter(0);           //设置重复计数次数
    PWMA_EnableUpdate();                //使能更新事件
    PWMA_UpdateRequestMode1();          //更新事件选择模式1

    PWMA_CC1PDisable();                 //关闭通道
    PWMA_CC1Output();                   //使能通道输出功能
    PWMA_CC1PEnable();                  //打开通道
    PWM_UpdateDuty(PWMA_CH1, 199);      //设置通道的PWM占空比
    PWMA_OC1REFPWMMode1();              //设置通道输出参考信号为PWM模式1信号
    PWMA_DisableOC1FastMode();          //关闭通道快速输出
    PWMA_BufferCCR1();                  //使能预装载，三相在更新事件同时生效
    PWMA_DisableETRFClearOC1REF();      //通道输出不受ETI信号影响
    PWMA_CC1PNonInverted();             //设置正极通道输出高电平有效
    PWMA_OC1PIdleLevelLow();            //空闲时正极通道输出低电平
    PWMA_EnablePWM1POutput();           //使能正极通道输出
    PWMA_DisablePWM1NOutput();          //关闭负极通道输出

    PWMA_CC2PDisable();                 //关闭通道
    PWMA_CC2Output();                   //使能通道输出功能
    PWMA_CC2PEnable();                  //打开通道
    PWM_UpdateDuty(PWMA_CH2, 199);      //设置通道的PWM占空比
    PWMA_OC2REFPWMMode1();              //设置通道输出参考信号为PWM模式1信号
    PWMA_DisableOC2FastMode();          //关闭通道快速输出
    PWMA_BufferCCR2();                  //使能预装载，三相在更新事件同时生效
    PWMA_DisableETRFClearOC2REF();      //通道输出不受ETI信号影响
    PWMA_CC2PNonInverted();             //设置正极通道输出高电平有效
    PWMA_OC2PIdleLevelLow();            //空闲时正极通道输出低电平
    PWMA_EnablePWM2POutput();           //使能正极通道输出
    PWMA_DisablePWM2NOutput();          //关闭负极通道输出

    PWMA_CC3PDisable();                 //关闭通道
    PWMA_CC3Output();                   //使能通道输出功能
    PWMA_CC3PEnable();                  //打开通道
    PWM_UpdateDuty(PWMA_CH3, 199);      //设置通道的PWM占空比
    PWMA_OC3REFPWMMode1();              //设置通道输出参考信号为PWM模式1信号
    PWMA_DisableOC3FastMode();          //关闭通道快速输出
    PWMA_BufferCCR3();                  //使能预装载，三相在更新事件同时生效
    PWMA_DisableETRFClearOC3REF();      //通道输出不受ETI信号影响
    PWMA_CC3PNonInverted();             //设置正极通道输出高电平有效
    PWMA_OC3PIdleLevelLow();            //空闲时正极通道输出低电平
    PWMA_EnablePWM3POutput();           //使能正极通道输出
    PWMA_DisablePWM3NOutput();          //关闭负极通道输出

    PWMA_CC4PDisable();                 //关闭通道
    PWMA_CC4Output();                   //使能通道输出功能
    PWMA_CC4PEnable();                  //打开通道
    PWM_UpdateDuty(PWMA_CH4, 380);      //设置通道的PWM占空比
    PWMA_OC4REFPWMMode2();              //设置通道输出参考信号为PWM模式2信号
    PWMA_DisableOC4FastMode();          //关闭通道快速输出
    PWMA_UnbufferCCR4();                //关闭CCRn预装载功能
    PWMA_DisableETRFClearOC4REF();      //通道输出不受ETI信号影响
    PWMA_DisablePWM4POutput();          //关闭正极通道输出
    PWMA_DisablePWM4NOutput();          //关闭负极通道输出

    PWMA_StandaloneMode();              //设置PWM定时器为独立模式
    PWMA_BufferCCOC();                  //使能捕获/比较控制位预装载功能
    PWMA_CCOCUpdateMode0();             //只在COMG为1时控制位才被更新
    PWMA_DisableTRGI();                 //关闭触发输入信号
    PWMA_PWM1PToTI();                   //关闭霍尔传感器控制功能
    PWMA_OC4REFAsTRGO();                //通道4的输出参考信号OC4REF作为TRGO
    PWMA_DisableExternalClock2();       //禁止外部时钟模式2
    PWMA_BRKDisable();                  //禁止刹车输入
    PWMA_DisableAutoOutput();           //禁止自动输出功能（只能软件设置MOE）
    PWMA_RunInactiveNoOutput();         //运行模式PWM不工作时禁止输出
    PWMA_IdleInactiveNoOutput();        //空闲模式PWM不工作时禁止输出
    PWMA_EnableMainOutput();            //使能PWM主输出
    PWMA_SetUnLock();                   //寄存器无写保护

    PWMA_Run();                         //PWMA开始运行

    //<<AICUBE_USER_PWM0_INITIAL_BEGIN>>
    // 在此添加用户初始化代码  
    //<<AICUBE_USER_PWM0_INITIAL_END>>
}

////////////////////////////////////////
// 设置PWM通道输出占空比
// 入口参数: PWMx: (目标PWM组和通道索引)
//           nCompare: (PWM占空比值)
// 函数返回: 无
////////////////////////////////////////
void PWM_UpdateDuty(uint8_t PWMx, uint16_t nCompare)
{
    switch (PWMx)
    {
    case PWMA_CH1:
        PWMA_SetCCR1Value(nCompare);    //设置通道比较值
        break;
    case PWMA_CH2:
        PWMA_SetCCR2Value(nCompare);    //设置通道比较值
        break;
    case PWMA_CH3:
        PWMA_SetCCR3Value(nCompare);    //设置通道比较值
        break;
    case PWMA_CH4:
        PWMA_SetCCR4Value(nCompare);    //设置通道比较值
        break;
    case PWMB_CH5:
        PWMB_SetCCR5Value(nCompare);    //设置通道比较值
        break;
    case PWMB_CH6:
        PWMB_SetCCR6Value(nCompare);    //设置通道比较值
        break;
    case PWMB_CH7:
        PWMB_SetCCR7Value(nCompare);    //设置通道比较值
        break;
    case PWMB_CH8:
        PWMB_SetCCR8Value(nCompare);    //设置通道比较值
        break;
    }
}

void PWM_UpdateDutyABC(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c)
{
    PWMA_SetCCR1Value(duty_a);
    PWMA_SetCCR2Value(duty_b);
    PWMA_SetCCR3Value(duty_c);
}

////////////////////////////////////////
// 读取PWM通道捕获值
// 入口参数: PWMx: (目标PWM组和通道索引)
// 函数返回: 捕获值
////////////////////////////////////////
uint16_t PWM_ReadCapture(uint8_t PWMx)
{
    uint16_t cap;

    switch (PWMx)
    {
    case PWMA_CH1:  cap = PWMA_ReadCCR1Value(); break;
    case PWMA_CH2:  cap = PWMA_ReadCCR2Value(); break;
    case PWMA_CH3:  cap = PWMA_ReadCCR3Value(); break;
    case PWMA_CH4:  cap = PWMA_ReadCCR4Value(); break;
    case PWMB_CH5:  cap = PWMB_ReadCCR5Value(); break;
    case PWMB_CH6:  cap = PWMB_ReadCCR6Value(); break;
    case PWMB_CH7:  cap = PWMB_ReadCCR7Value(); break;
    case PWMB_CH8:  cap = PWMB_ReadCCR8Value(); break;
    default: cap = 0;
    }

    return cap;
}

////////////////////////////////////////
// 设置高速模式PWM通道输出占空比
// 入口参数: PWMx: (目标PWM组和通道索引)
//           nCompare: (PWM占空比值)
// 函数返回: 无
////////////////////////////////////////
void HSPWM_UpdateDuty(uint8_t PWMx, uint16_t nCompare)
{
    switch (PWMx)
    {
    case PWMA_CH1:
        HSPWMA_SetCCR1Value(nCompare);  //设置通道比较值
        break;
    case PWMA_CH2:
        HSPWMA_SetCCR2Value(nCompare);  //设置通道比较值
        break;
    case PWMA_CH3:
        HSPWMA_SetCCR3Value(nCompare);  //设置通道比较值
        break;
    case PWMA_CH4:
        HSPWMA_SetCCR4Value(nCompare);  //设置通道比较值
        break;
    case PWMB_CH5:
        HSPWMB_SetCCR5Value(nCompare);  //设置通道比较值
        break;
    case PWMB_CH6:
        HSPWMB_SetCCR6Value(nCompare);  //设置通道比较值
        break;
    case PWMB_CH7:
        HSPWMB_SetCCR7Value(nCompare);  //设置通道比较值
        break;
    case PWMB_CH8:
        HSPWMB_SetCCR8Value(nCompare);  //设置通道比较值
        break;
    }
}

////////////////////////////////////////
// 读取高速模式PWM通道捕获值
// 入口参数: PWMx: (目标PWM组和通道索引)
// 函数返回: 捕获值
////////////////////////////////////////
uint16_t HSPWM_ReadCapture(uint8_t PWMx)
{
    uint16_t cap;

    switch (PWMx)
    {
    case PWMA_CH1:  cap = HSPWMA_ReadCCR1Value();   break;
    case PWMA_CH2:  cap = HSPWMA_ReadCCR2Value();   break;
    case PWMA_CH3:  cap = HSPWMA_ReadCCR3Value();   break;
    case PWMA_CH4:  cap = HSPWMA_ReadCCR4Value();   break;
    case PWMB_CH5:  cap = HSPWMB_ReadCCR5Value();   break;
    case PWMB_CH6:  cap = HSPWMB_ReadCCR6Value();   break;
    case PWMB_CH7:  cap = HSPWMB_ReadCCR7Value();   break;
    case PWMB_CH8:  cap = HSPWMB_ReadCCR8Value();   break;
    default: cap = 0;
    }

    return cap;
}

////////////////////////////////////////
// 异步方式读取PWMA特殊功能寄存器
// 入口参数: addr: (PWMA特殊功能寄存器的低7位)
// 函数返回: 寄存器的值
////////////////////////////////////////;
uint8_t HSPWMA_ReadReg(uint8_t addr)
{
    uint8_t dat;

    while (HSPWMA_CheckBusy());         //等待前一个异步读写完成
    HSPWMA_AsyncRead(addr, dat);        //触发异步方式读取寄存器

    return dat;
}

////////////////////////////////////////
// 异步方式写PWMA特殊功能寄存器
// 入口参数: addr: (PWMA特殊功能寄存器的低7位)
//           dat: (待写入的数据)
// 函数返回: 无
////////////////////////////////////////;
void HSPWMA_WriteReg(uint8_t addr, uint8_t dat)
{
    while (HSPWMA_CheckBusy());         //等待前一个异步读写完成
    HSPWMA_AsyncWrite(addr, dat);       //触发异步方式写寄存器
}

////////////////////////////////////////
// 异步方式读取PWMB特殊功能寄存器
// 入口参数: addr: (PWMB特殊功能寄存器的低7位)
// 函数返回: 寄存器的值
////////////////////////////////////////;
uint8_t HSPWMB_ReadReg(uint8_t addr)
{
    uint8_t dat;

    while (HSPWMB_CheckBusy());         //等待前一个异步读写完成
    HSPWMB_AsyncRead(addr, dat);        //触发异步方式读取寄存器

    return dat;
}

////////////////////////////////////////
// 异步方式写PWMB特殊功能寄存器
// 入口参数: addr: (PWMB特殊功能寄存器的低7位)
//           dat: (待写入的数据)
// 函数返回: 无
////////////////////////////////////////;
void HSPWMB_WriteReg(uint8_t addr, uint8_t dat)
{
    while (HSPWMB_CheckBusy());         //等待前一个异步读写完成
    HSPWMB_AsyncWrite(addr, dat);       //触发异步方式写寄存器
}



//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码  
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


#define PWMA_NORMAL_RELOAD        399u
#define PWMA_NORMAL_DUTY          199u
#define MOTOR_PULSE_MIN_HZ        500u
#define MOTOR_PULSE_MAX_HZ        3000u
#define MOTOR_PULSE_MIN_WIDTH_PM  125u
#define MOTOR_PULSE_MAX_WIDTH_PM  500u
#define MOTOR_PULSE_MIN_AMPLITUDE 12u
#define MOTOR_PULSE_ENVELOPE_MS   20u

static void Motor_StopTone(void)
{
    PWMA_Stop();
    PWM_UpdateDutyABC(0u, 0u, 0u);
    PWMA_GenerateUpdateEvent();
}

/* A-B 产生受控线电压，C 相保持中点；底层仍使用 120 kHz 功率载波。 */
static void Motor_SetPulseLevel(int16_t level)
{
    uint16_t duty_a;
    uint16_t duty_b;

    duty_a = (uint16_t)((int16_t)PWMA_NORMAL_DUTY + level);
    duty_b = (uint16_t)((int16_t)PWMA_NORMAL_DUTY - level);
    PWM_UpdateDutyABC(duty_a, duty_b, PWMA_NORMAL_DUTY);
}

/*
 * 8-bit pulse wave。非 50% 脉宽时降低负半周幅度，使每周期平均线电压为零，
 * 在保留 25%/37.5% pulse 音色的同时减少直流力矩和转子顶动。
 */
static void Motor_PlayPulseTone(uint16_t target_hz, uint16_t duration_ms,
                                uint16_t pulse_width_pm, uint16_t max_amplitude)
{
    uint16_t period_us;
    uint16_t positive_us;
    uint16_t negative_us;
    uint16_t cycle_count;
    uint16_t attack_cycles;
    uint16_t release_cycles;
    uint16_t cycles_left;
    uint16_t amplitude;
    uint16_t negative_amplitude;
    uint16_t cycle;

    if ((target_hz < MOTOR_PULSE_MIN_HZ) || (target_hz > MOTOR_PULSE_MAX_HZ) ||
        (pulse_width_pm < MOTOR_PULSE_MIN_WIDTH_PM) ||
        (pulse_width_pm > MOTOR_PULSE_MAX_WIDTH_PM) ||
        (max_amplitude > PWMA_NORMAL_DUTY) ||
        (max_amplitude < MOTOR_PULSE_MIN_AMPLITUDE) || (duration_ms < 40u))
    {
        return;
    }

    period_us = (uint16_t)(1000000UL / target_hz);
    positive_us = (uint16_t)((uint32_t)period_us * pulse_width_pm / 1000u);
    negative_us = period_us - positive_us;
    cycle_count = (uint16_t)((uint32_t)target_hz * duration_ms / 1000UL);
    attack_cycles = (uint16_t)((uint32_t)target_hz * MOTOR_PULSE_ENVELOPE_MS /
                               1000UL);
    release_cycles = attack_cycles;
    if (attack_cycles == 0u)
    {
        attack_cycles = 1u;
        release_cycles = 1u;
    }

    PWMA_Stop();
    PWMA_SetReload16(PWMA_NORMAL_RELOAD);
    PWMA_SetCounter(0u);
    PWM_UpdateDutyABC(PWMA_NORMAL_DUTY, PWMA_NORMAL_DUTY, PWMA_NORMAL_DUTY);
    PWMA_GenerateUpdateEvent();
    PWMA_Run();

    for (cycle = 0u; cycle < cycle_count; cycle++)
    {
        cycles_left = cycle_count - cycle;
        if (cycle < attack_cycles)
        {
            amplitude = (uint16_t)(MOTOR_PULSE_MIN_AMPLITUDE +
                        (uint32_t)(max_amplitude - MOTOR_PULSE_MIN_AMPLITUDE) *
                        cycle / attack_cycles);
        }
        else if (cycles_left <= release_cycles)
        {
            amplitude = (uint16_t)(MOTOR_PULSE_MIN_AMPLITUDE +
                        (uint32_t)(max_amplitude - MOTOR_PULSE_MIN_AMPLITUDE) *
                        (cycles_left - 1u) / release_cycles);
        }
        else
        {
            amplitude = max_amplitude;
        }

        negative_amplitude = (uint16_t)((uint32_t)amplitude * pulse_width_pm /
                             (1000u - pulse_width_pm));

        Motor_SetPulseLevel((int16_t)amplitude);
        delay_us(positive_us);
        Motor_SetPulseLevel(-(int16_t)negative_amplitude);
        delay_us(negative_us);
    }

    Motor_StopTone();
}

////////////////////////////////////////
// 开机电机自检提示音 
// (在主函数中调用，不要放在PWMA_Init内部，否则会导致无限递归或状态混乱)
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void Motor_PowerOn_SelfTest(void)
{
    Motor_StopTone();
    delay_ms(50u);

    /* Do-Mi-Sol：脉宽逐渐变宽，音色从明亮变为饱满的经典方波。 */
    Motor_PlayPulseTone(1047u, 160u, 250u, 100u); /* C6, 25% pulse */
    delay_ms(45u);
    Motor_PlayPulseTone(1319u, 160u, 375u, 100u); /* E6, 37.5% pulse */
    delay_ms(45u);
    Motor_PlayPulseTone(1568u, 300u, 500u, 100u); /* G6, 50% square */

    /* 恢复 FOC 使用的 120 kHz PWM 中点，不重复初始化 GPIO/输出级。 */
    PWMA_SetReload16(PWMA_NORMAL_RELOAD);
    PWMA_SetCounter(0u);
    PWM_UpdateDutyABC(PWMA_NORMAL_DUTY, PWMA_NORMAL_DUTY, PWMA_NORMAL_DUTY);
    PWMA_GenerateUpdateEvent();
    PWMA_Run();
}
