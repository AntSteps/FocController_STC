//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: spi.c
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


BOOL fSPITransBusy;                     //SPI读写忙标志

////////////////////////////////////////
// SPI初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SPI_Init(void)
{
    SPI_SwitchP3n();                    //选择SPI数据口: SS(P3.5), MOSI(P3.4), MISO(P3.3), SCLK(P3.2)

    SPI_MasterMode();                   //设置SPI为主机模式
    SPI_IgnoreSS();                     //忽略SS脚
    SPI_DataMSB();                      //设置SPI数据顺序为MSB (高位在前)
    SPI_SetMode0();                     //设置SPI工作模式0 (CPOL=0, CPHA=0)
    SPI_SetClockDivider4();             //设置SPI时钟分频

    HSSPI_Disable();                    //关闭SPI高速模式

    SPI_SetIntPriority(0);              //设置中断为最低优先级
    SPI_EnableInt();                    //使能SPI中断
    fSPITransBusy = 0;                  //清除SPI忙标志位

    SPI_Enable();                       //使能SPI功能

    //<<AICUBE_USER_SPI_INITIAL_BEGIN>>
    // 在此添加用户初始化代码  
    //<<AICUBE_USER_SPI_INITIAL_END>>
}

////////////////////////////////////////
// SPI主机模式发送字节函数
// 入口参数: dat (待发送的字节数据)
// 函数返回: 无
////////////////////////////////////////
void SPI_WriteByte(uint8_t dat)
{
    fSPITransBusy = 1;                  //设置发送忙标志
    SPI_SendData(dat);                  //触发主机发送数据
    while (fSPITransBusy);              //等待发送完成
}

////////////////////////////////////////
// SPI主机模式读取字节函数
// 入口参数: 无
// 函数返回: 读取的字节数据
////////////////////////////////////////
uint8_t SPI_ReadByte(void)
{
    fSPITransBusy = 1;                  //设置读取忙标志
    SPI_SendData(0xff);                 //触发主机读取数据（主机发送时钟信号）
    while (fSPITransBusy);              //等待读取完成
    return SPI_ReadData();
}


////////////////////////////////////////
// SPI中断服务程序
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SPI_ISR(void) interrupt SPI_VECTOR
{
    //<<AICUBE_USER_SPI_ISR_CODE1_BEGIN>>
    // 在此添加中断函数用户代码  
    if (SPI_CheckFlag())                //判断SPI中断
    {
        SPI_ClearFlag();                //清除SPI中断标志
        fSPITransBusy = 0;              //清除SPI忙标志位
    }
    //<<AICUBE_USER_SPI_ISR_CODE1_END>>
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码  
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


