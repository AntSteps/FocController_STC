//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: adc.c
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

// 母线电压采集配置
#define ADC_VREF        5.0f            // ADC参考电压(V)
#define ADC_RESOLUTION  4096.0f         // 12位ADC分辨率
#define VIN_DIVIDER_R1  10000.0f        // 分压电阻R9 (10K)
#define VIN_DIVIDER_R2  10000.0f        // 分压电阻R12 (10K)

// 计算母线电压系数: Vref / 4096 * (R1+R2)/R2
#define VIN_SCALE_FACTOR (ADC_VREF / ADC_RESOLUTION * ((VIN_DIVIDER_R1 + VIN_DIVIDER_R2) / VIN_DIVIDER_R2))

#define BUS_VOLTAGE_ADC_CH  1           // 母线电压采集通道

static uint16_t s_busVoltageRaw = 0;    // 母线电压ADC原始值
static float s_busVoltage = 0.0f;       // 母线电压值(V)

//<<AICUBE_USER_GLOBAL_DEFINE_END>>


BOOL fADCConverted;                     //ADC转换完成标志

////////////////////////////////////////
// ADC初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void ADC_Init(void)
{
    ADC_SetClockDivider(0);             //设置ADC时钟
    ADC_ResultRightAlign();             //设置ADC结果右对齐(12位结果)
    // ADC_EnablePWMTrig();             //禁用PWM触发，使用软件手动触发
    ADC_SetRepeat16Times();             //使能ADC自动重复转换16次，提高采样精度

    ADC_SetCSSetupCycles(2);            //增加ADC通道选择建立时间
    ADC_SetCSHoldCycles(3);             //增加ADC通道选择保持时间
    ADC_SetSampleDutyCycles(63);        //增加ADC采样时间到最大(63个时钟周期)，让采样更平稳

    ADC_DisableETR();                   //禁止ADC外部触发功能

    ADC_SetIntPriority(0);              //设置中断为最低优先级
    ADC_EnableInt();                    //使能ADC中断
    fADCConverted = 0;                  //初始化转换结束标志

    ADC_ActiveChannel(BUS_VOLTAGE_ADC_CH);  //选择母线电压采集通道
    ADC_Enable();                       //使能ADC功能
    ADC_Start();                        //启动ADC转换，自动连续采集

    //<<AICUBE_USER_ADC_INITIAL_BEGIN>>
    // 在此添加用户初始化代码
    //<<AICUBE_USER_ADC_INITIAL_END>>
}

////////////////////////////////////////
// 获取ADC转换结果函数
// 入口参数: ch (ADC通道选择)
// 函数返回: ADC转换结果
////////////////////////////////////////
uint16_t ADC_Convert(uint8_t ch)
{
    uint16_t res;                       //定义保存ADC结果的变量

    ADC_ActiveChannel(ch);              //选择ADC通道
    ADC_Start();                        //开始ADC转换
    while (!fADCConverted);             //等待ADC转换完成
    fADCConverted = 0;                  //清除ADC转换结束标志
    res = ADC_ReadResult();             //读取ADC转换结果

    return res;                         //返回ADC结果
}


////////////////////////////////////////
// ADC中断服务程序
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void ADC_ISR(void) interrupt ADC_VECTOR
{
    //<<AICUBE_USER_ADC_ISR_CODE1_BEGIN>>
    // 在此添加中断函数用户代码
    ADC_ClearFlag();                    //清除ADC转换完成中断标志
    fADCConverted = 1;                  //设置转换结束标志

    // 读取母线电压ADC值并转换为电压
    s_busVoltageRaw = ADC_ReadResult();
    s_busVoltage = s_busVoltageRaw * VIN_SCALE_FACTOR;

    // 自动重启下一次转换，实现连续采集
    ADC_Start();
    //<<AICUBE_USER_ADC_ISR_CODE1_END>>
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码

////////////////////////////////////////
// 获取母线电压函数
// 入口参数: 无
// 函数返回: 母线电压值(V)
////////////////////////////////////////
float ADC_GetBusVoltage(void)
{
    return s_busVoltage;
}

////////////////////////////////////////
// 获取母线电压ADC原始值
// 入口参数: 无
// 函数返回: ADC原始值(0-4095)
////////////////////////////////////////
uint16_t ADC_GetBusVoltageRaw(void)
{
    return s_busVoltageRaw;
}

//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


