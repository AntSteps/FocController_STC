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
    ADC_EnablePWMTrig();                //使能PWM同步触发ADC
    ADC_DisableRepeatConv();            //关闭ADC自动重复转换功能

    ADC_SetCSSetupCycles(0);            //设置ADC通道选择建立时间
    ADC_SetCSHoldCycles(1);             //设置ADC通道选择保持时间
    ADC_SetSampleDutyCycles(9);         //设置ADC通道采样时间

    ADC_DisableETR();                   //禁止ADC外部触发功能

    ADC_SetIntPriority(0);              //设置中断为最低优先级
    ADC_EnableInt();                    //使能ADC中断
    fADCConverted = 0;                  //初始化转换结束标志

    ADC_ActiveChannel(1);               //选择ADC通道
    ADC_Enable();                       //使能ADC功能

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
    //<<AICUBE_USER_ADC_ISR_CODE1_END>>
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码  
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


