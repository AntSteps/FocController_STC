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

//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
static uint8_t xdata s_uart3_rx_buffers[UART3_RX_BUFFER_COUNT][UART3_RX_DMA_BUFFER_SIZE];
static volatile uint8_t s_uart3_rx_len[UART3_RX_BUFFER_COUNT] = {0, 0};
static volatile uint8_t s_uart3_rx_active = 0;
static volatile uint8_t s_uart3_rx_ready = UART3_RX_BUFFER_COUNT;
static volatile uint8_t s_uart3_rx_locked = 0;
static volatile uint8_t s_uart3_tx_ready = 1;
//<<AICUBE_USER_GLOBAL_DEFINE_END>>


uint8_t xdata pu8UR3DMATxBuffer[UART3_TX_DMA_BUFFER_SIZE];    //串口3DMA发送缓冲区数组

#ifndef UART3_VECTOR
#define UART3_VECTOR 17
#endif

static void UART3_RestartRx(uint8_t next_index)
{
    DMA_UART3_ClearRxFlag();
    DMA_UART3_ClearFIFO();
    UART3_ClearTimeoutFlag();
    DMA_UART3_SetRxAddress(s_uart3_rx_buffers[next_index]);
    DMA_UART3_TriggerRx();
}

static uint8_t UART3_GetRxCount(void)
{
    uint16_t rx_done;

    rx_done = DMA_UART3_ReadRxDone();
    if (rx_done >= UART3_RX_DMA_BUFFER_SIZE)
    {
        return UART3_RX_DMA_BUFFER_SIZE;
    }
    return (uint8_t)(rx_done + 1u);
}

static void UART3_PublishRx(uint8_t rx_len)
{
    uint8_t completed_index;
    uint8_t next_index;

    if (rx_len == 0)
    {
        UART3_RestartRx(s_uart3_rx_active);
        return;
    }

    completed_index = s_uart3_rx_active;
    next_index = (uint8_t)(completed_index ^ 1u);

    if ((s_uart3_rx_ready == next_index) && s_uart3_rx_locked)
    {
        UART3_RestartRx(completed_index);
        return;
    }

    s_uart3_rx_len[completed_index] = rx_len;
    s_uart3_rx_ready = completed_index;
    s_uart3_rx_active = next_index;
    UART3_RestartRx(next_index);
}

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

    DMA_UART3_SetTxAmount(UART3_TX_DMA_BUFFER_SIZE - 1);          //设置串口DMA发送总字节数
    DMA_UART3_SetTxAddress(pu8UR3DMATxBuffer); //设置串口DMA发送缓冲区地址
    DMA_UART3_SetInterval(0);           //设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearTxFlag();            //清除串口发送DMA中断标志
    DMA_UART3_SetTxBusPriority(0);      //设置总线访问为最低优先级
    DMA_UART3_SetTxIntPriority(0);      //设置中断为最低优先级
    DMA_UART3_EnableTxInt();            //使能串口3DMA发送中断
    DMA_UART3_EnableTx();               //使能串口3DMA发送功能
//  DMA_UART3_TriggerTx();              //触发串口3DMA发送

    DMA_UART3_SetRxAmount(UART3_RX_DMA_BUFFER_SIZE - 1);          //设置串口DMA接收总字节数
    DMA_UART3_SetRxAddress(s_uart3_rx_buffers[0]); //设置串口DMA接收缓冲区地址
    DMA_UART3_SetInterval(0);           //设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearFIFO();              //清空串口DMA接收FIFO缓冲区
    DMA_UART3_ClearRxFlag();            //清除串口接收DMA中断标志
    DMA_UART3_SetRxBusPriority(0);      //设置总线访问为最低优先级
    DMA_UART3_SetRxIntPriority(0);      //设置中断为最低优先级
    DMA_UART3_EnableRxInt();            //使能串口3DMA接收中断
    DMA_UART3_EnableRx();               //使能串口3DMA接收功能
//  DMA_UART3_TriggerRx();              //触发串口3DMA接收

    UART3_TimeoutScale_BRT();
    UART3_SetTimeoutInterval(16);
    UART3_ClearTimeoutFlag();
    UART3_EnableTimeout();
    UART3_EnableTimeoutInt();
    UART3_EnableInt();
    DMA_UART3_TriggerRx();

    UART3_ClearTxFlag();
    s_uart3_tx_ready = 1;

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
    //注释掉以屏蔽printf发送
    // while (!s_uart3_tx_ready);
    // s_uart3_tx_ready = 0;
    // UART3_SendData(dat);

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
    if (DMA_UART3_CheckRxFlag())        //判断串口接收DMA中断
    {
        UART3_PublishRx(UART3_RX_DMA_BUFFER_SIZE);
    }
    //<<AICUBE_USER_UART3_ISR_CODE3_END>>
}

void UART3_ISR(void) interrupt UART3_VECTOR
{
    if (UART3_CheckTxFlag())
    {
        UART3_ClearTxFlag();
        s_uart3_tx_ready = 1;
    }

    if (UART3_CheckTimeoutFlag())
    {
        UART3_PublishRx(UART3_GetRxCount());
    }

    if (UART3_CheckRxFlag())
    {
        UART3_ClearRxFlag();
    }
}


//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
uint8_t xdata *UART_GetRxBuffer(uint8_t *len)
{
    uint8_t ready_index;

    ready_index = s_uart3_rx_ready;
    if (ready_index >= UART3_RX_BUFFER_COUNT)
    {
        *len = 0;
        return (uint8_t xdata *)0;
    }

    s_uart3_rx_locked = 1;
    *len = s_uart3_rx_len[ready_index];
    return (uint8_t xdata *)s_uart3_rx_buffers[ready_index];
}

void UART_ReleaseRxBuffer(void)
{
    s_uart3_rx_ready = UART3_RX_BUFFER_COUNT;
    s_uart3_rx_locked = 0;
}

//! 在改变dma数组的时候注意修改len的值
void UART_DMA_Send(uint8_t *pData, uint8_t len)
{
    if(len > UART3_TX_DMA_BUFFER_SIZE) len = UART3_TX_DMA_BUFFER_SIZE;
    if (!DMA_UART3_CheckTxFlag())
    {
        memset(pu8UR3DMATxBuffer, 0x00, sizeof(pu8UR3DMATxBuffer));
        memcpy(pu8UR3DMATxBuffer, pData, len);
        DMA_UART3_TriggerTx();
    }
}
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


