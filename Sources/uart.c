//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: uart.c
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


uint8_t xdata pu8UR3DMATxBuffer[21];    //串口3DMA发送缓冲区数组
uint8_t xdata pu8UR3DMARxBuffer[17];    //串口3DMA接收缓冲区数组

////////////////////////////////////////
// 串口3初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void UART3_Init(void)
{
    UART3_SwitchP5051();                //设置串口数据端口: RxD3 (P5.0), TxD3 (P5.1)

    UART3_Timer3BRT();                  //选择定时器3作为串口3波特率发生器

    UART3_EnableRx();                   //使能串口3接收数据
    UART3_Mode0();                      //设置串口3为模式0 (8位数据可变波特率)

    DMA_UART3_SetTxAmount(20);          //设置串口DMA发送总字节数
    DMA_UART3_SetTxAddress(pu8UR3DMATxBuffer); //设置串口DMA发送缓冲区地址
    DMA_UART3_SetInterval(0);           //设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearTxFlag();            //清除串口发送DMA中断标志
    DMA_UART3_SetTxBusPriority(0);      //设置总线访问为最低优先级
    DMA_UART3_SetTxIntPriority(0);      //设置中断为最低优先级
    DMA_UART3_EnableTxInt();            //使能串口3DMA发送中断
    DMA_UART3_EnableTx();               //使能串口3DMA发送功能
//  DMA_UART3_TriggerTx();              //触发串口3DMA发送

    DMA_UART3_SetRxAmount(16);          //设置串口DMA接收总字节数
    DMA_UART3_SetRxAddress(pu8UR3DMARxBuffer); //设置串口DMA接收缓冲区地址
    DMA_UART3_SetInterval(0);           //设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearFIFO();              //清空串口DMA接收FIFO缓冲区
    DMA_UART3_ClearRxFlag();            //清除串口接收DMA中断标志
    DMA_UART3_SetRxBusPriority(0);      //设置总线访问为最低优先级
    DMA_UART3_SetRxIntPriority(0);      //设置中断为最低优先级
    DMA_UART3_EnableRxInt();            //使能串口3DMA接收中断
    DMA_UART3_EnableRx();               //使能串口3DMA接收功能
//  DMA_UART3_TriggerRx();              //触发串口3DMA接收

    UART3_SetTxFlag();                  //设置发送标志以配合printf函数

    //<<AICUBE_USER_UART3_INITIAL_BEGIN>>
    // 在此添加用户初始化代码  
    //<<AICUBE_USER_UART3_INITIAL_END>>
}

////////////////////////////////////////
// 重写printf字符发送重定向函数
// 入口参数: dat (printf函数待打印的字符)
// 函数返回: 需要返回入口参数的数据
////////////////////////////////////////
char putchar (char dat)                 //将串口3和printf函数绑定
{
    while (!UART3_CheckTxFlag());
    UART3_ClearTxFlag();
    UART3_SendData(dat);

    return dat;
}


////////////////////////////////////////
// 串口3发送DMA中断服务程序
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void DMA_UART3TX_ISR(void) interrupt DMA_UR3T_VECTOR
{
    //<<AICUBE_USER_UART3_ISR_CODE2_BEGIN>>
    // 在此添加中断函数用户代码  
    if (DMA_UART3_CheckTxFlag())        //判断串口发送DMA中断
    {
        DMA_UART3_ClearTxFlag();        //清除串口发送DMA中断标志
    }
    //<<AICUBE_USER_UART3_ISR_CODE2_END>>
}

////////////////////////////////////////
// 串口3接收DMA中断服务程序
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void DMA_UART3RX_ISR(void) interrupt DMA_UR3R_VECTOR
{
    //<<AICUBE_USER_UART3_ISR_CODE3_BEGIN>>
    // 在此添加中断函数用户代码  
    if (DMA_UART3_CheckRxFlag())        //判断串口接收DMA中断
    {
        DMA_UART3_ClearRxFlag();        //清除串口接收DMA中断标志
    }
    //<<AICUBE_USER_UART3_ISR_CODE3_END>>
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码  
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


