
#include "config.h"

sbit RS485_DIR = P2 ^ 5;
sbit UART_DEBUG = P0 ^ 3;
#define UART3_RX_SLOT_COUNT  4u
#define UART3_INVALID_SLOT   0xFFu
static uint8_t xdata s_uart3_rx_dma_buffer[UART3_RX_SLOT_COUNT][UART3_RX_DMA_BUFFER_SIZE];
static volatile uint8_t s_uart3_rx_len[UART3_RX_SLOT_COUNT] = {0};
static volatile uint8_t s_rx_active_index = 0; // 当前接收的
static volatile uint8_t s_rx_publish_index = UART3_INVALID_SLOT;
static volatile uint8_t s_rx_pending_count = 0u;
static volatile uint8_t s_rx_read_index = UART3_INVALID_SLOT;
static volatile BOOL s_rx_locked = 0;
static volatile uint8_t s_uart3_rx_activity = 0u;
static volatile uint8_t s_uart3_timeout_retry = 0u;
static volatile uint8_t s_uart3_tx_ready = 1; // 串口3发送DMA是否准备好
static void UART_DebugPulse(uint8_t count)
{
    while (count--)
    {
        P36 = 1;
        delay_us(20);
        P36 = 0;
        delay_us(20);
    }
}

uint8_t xdata pu8UR3DMATxBuffer[UART3_TX_DMA_BUFFER_SIZE]; // 串口3DMA发送缓冲区数组

#ifndef UART3_VECTOR
#define UART3_VECTOR 17
#endif

static uint8_t UART3_TryLockTx(void)
{
    uint8_t ea_state;

    ea_state = EA;
    DisableGlobalInt();
    if (!s_uart3_tx_ready)
    {
        EA = ea_state;
        return 0;
    }

    s_uart3_tx_ready = 0;
    EA = ea_state;
    return 1;
}

static uint8_t UART3_WaitLockTx(void)
{
    uint16_t wait_us;

    wait_us = 30000;
    while (wait_us--)
    {
        if (UART3_TryLockTx())
        {
            return 1;
        }
        delay_us(1);
    }

    return 0;
}

static uint8_t UART3_FindFreeRxSlot(uint8_t start_index)
{
    uint8_t index;
    uint8_t count;

    index = start_index;
    for (count = 0u; count < UART3_RX_SLOT_COUNT; count++)
    {
        if (index >= UART3_RX_SLOT_COUNT)
        {
            index = 0u;
        }

        if (s_uart3_rx_len[index] == 0u)
        {
            return index;
        }

        index++;
    }

    return UART3_INVALID_SLOT;
}

static uint8_t UART3_FindNextPendingSlot(uint8_t start_index)
{
    uint8_t index;
    uint8_t count;

    index = start_index;
    for (count = 0u; count < UART3_RX_SLOT_COUNT; count++)
    {
        if (index >= UART3_RX_SLOT_COUNT)
        {
            index = 0u;
        }

        if ((s_uart3_rx_len[index] != 0u) &&
            ((!s_rx_locked) || (index != s_rx_read_index)))
        {
            return index;
        }

        index++;
    }

    return UART3_INVALID_SLOT;
}

static void UART3_ClearRxSlot(uint8_t slot_index)
{
    memset(s_uart3_rx_dma_buffer[slot_index], 0x00, UART3_RX_DMA_BUFFER_SIZE);
}

static void UART3_ResetRxState(void)
{
    uint8_t index;

    s_rx_active_index = 0u;
    s_rx_publish_index = UART3_INVALID_SLOT;
    s_rx_pending_count = 0u;
    s_rx_read_index = UART3_INVALID_SLOT;
    s_rx_locked = 0;
    s_uart3_rx_activity = 0u;
    s_uart3_timeout_retry = 0u;

    for (index = 0u; index < UART3_RX_SLOT_COUNT; index++)
    {
        s_uart3_rx_len[index] = 0u;
        UART3_ClearRxSlot(index);
    }
}

// static void UART3_RestartRx(void)
// {
//     s_uart3_rx_activity = 0u;
//     s_uart3_timeout_retry = 0u;
//     UART3_ClearRxSlot(s_rx_active_index);
//     DMA_UART3_ClearRxFlag();
//     DMA_UART3_ClearFIFO();
//     DMA_UART3_SetRxAddress(s_uart3_rx_dma_buffer[s_rx_active_index]);
//     DMA_UART3_TriggerRx();
// }
static void UART3_RestartRx(void)
{
    s_uart3_rx_activity = 0u;
    s_uart3_timeout_retry = 0u;
    
    // 关闭 DMA，复位内部状态
    DMA_UART3_DisableRx(); 

    // 清除中断标志和底层 FIFO 里的残余字节
    DMA_UART3_ClearRxFlag();
    DMA_UART3_ClearFIFO();
    
    // 重新装载地址和最大长度
    DMA_UART3_SetRxAddress(s_uart3_rx_dma_buffer[s_rx_active_index]);
    DMA_UART3_SetRxAmount(UART3_RX_DMA_BUFFER_SIZE - 1); 
    
    // 重新启动
    DMA_UART3_EnableRx();  
    DMA_UART3_TriggerRx();
}
static void UART3_PublishRx(uint8_t rx_len)
{
    uint8_t publish_index;
    uint8_t next_active_index;

    if (rx_len == 0)
    {
        UART3_RestartRx();
        return;
    }
    if (rx_len > UART3_RX_DMA_BUFFER_SIZE)
    {
        rx_len = UART3_RX_DMA_BUFFER_SIZE;
    }

    publish_index = s_rx_active_index;
    next_active_index = UART3_FindFreeRxSlot((uint8_t)(publish_index + 1u));
    if ((next_active_index == UART3_INVALID_SLOT) || (next_active_index == publish_index))
    {
        UART3_RestartRx();
        return;
    }

    s_uart3_rx_len[publish_index] = rx_len;
    if (s_rx_pending_count == 0u)
    {
        s_rx_publish_index = publish_index;
    }
    s_rx_pending_count++;
    s_uart3_timeout_retry = 0u;
    s_rx_active_index = next_active_index;
    UART3_RestartRx();
}

////////////////////////////////////////
// 串口3初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void UART3_Init(void)
{
    UART3_ResetRxState();
    UART3_SwitchP5051(); // 设置串口数据端口: RxD3 (P5.0), TxD3 (P5.1)
    SetP5nAutoMode(PIN_0 | PIN_1);

    UART3_Timer3BRT(); // 选择定时器3作为串口3波特率发生器

    UART3_EnableRx(); // 使能串口3接收数据
    UART3_Mode0();    // 设置串口3为模式0 (8位数据可变波特率)

    DMA_UART3_SetTxAmount(UART3_TX_DMA_BUFFER_SIZE - 1); // 设置串口DMA发送总字节数
    DMA_UART3_SetTxAddress(pu8UR3DMATxBuffer);           // 设置串口DMA发送缓冲区地址
    DMA_UART3_SetInterval(0);                            // 设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearTxFlag();                             // 清除串口发送DMA中断标志
    DMA_UART3_SetTxBusPriority(2);                       // 发送DMA保持较高优先级，避免回包尾部被拖延
    DMA_UART3_SetTxIntPriority(3);                       // 发送完成需要尽快切回接收
    DMA_UART3_EnableTxInt();                             // 使能串口3DMA发送中断
    DMA_UART3_EnableTx();                                // 使能串口3DMA发送功能

    DMA_UART3_SetRxAmount(UART3_RX_DMA_BUFFER_SIZE - 1);              // 设置串口DMA接收总字节数
    DMA_UART3_SetRxAddress(s_uart3_rx_dma_buffer[s_rx_active_index]); // 设置串口DMA接收缓冲区地址
    DMA_UART3_SetInterval(0);                                         // 设置串口DMA发送/接收字节间隔时间（系统时钟）
    DMA_UART3_ClearFIFO();                                            // 清空串口DMA接收FIFO缓冲区
    DMA_UART3_ClearRxFlag();                                          // 清除串口接收DMA中断标志
    DMA_UART3_SetRxBusPriority(3);                                    // 接收DMA优先级尽量提高，避免被FOC更新打断
    DMA_UART3_SetRxIntPriority(3);                                    // 接收完成优先处理，降低丢命令概率
    DMA_UART3_EnableRxInt();                                          // 使能串口3DMA接收中断
    DMA_UART3_EnableRx();                                             // 使能串口3DMA接收功能
    DMA_UART3_TriggerRx();

    // 串口总线空闲中断
    UART3_TimeoutScale_BRT();
    // 发送相邻字节之间可能有抖动，超时间隔太短会把一帧拆成多段。
    UART3_SetTimeoutInterval(128);
    UART3_ClearTimeoutFlag();
    UART3_EnableTimeout();
    UART3_EnableTimeoutInt();
    UART3_SetIntPriority(1); // 总线空闲中断高于TIMER0，避免收满一帧后迟迟得不到处理
    UART3_EnableInt();
    UART3_ClearTxFlag();

    RS485_DIR = 0; // 默认使能RS485接收

    UART_DEBUG = 0;
}

////////////////////////////////////////
// 重写printf字符发送重定向函数
// 入口参数: dat (printf函数待打印的字符)
// 函数返回: 需要返回入口参数的数据
////////////////////////////////////////
char putchar(char dat) // 将串口3和printf函数绑定
{
    return dat;
}

static uint8_t UART3_GetRxCount(void)
{
    uint16_t rx_done;

    rx_done = DMA_UART3_ReadRxDone();
    if (rx_done == 0)
    {
        return 0;
    }
    if (rx_done >= UART3_RX_DMA_BUFFER_SIZE)
    {
        return UART3_RX_DMA_BUFFER_SIZE;
    }
    return (uint8_t)rx_done;
}
////////////////////////////////////////
// 串口3发送DMA中断服务程序
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void DMA_UART3TX_ISR(void) interrupt DMA_UR3T_VECTOR
{
    //<<AICUBE_USER_UART3_ISR_CODE2_BEGIN>>
    if (DMA_UART3_CheckTxFlag()) // 判断串口发送DMA中断
    {
        DMA_UART3_ClearTxFlag(); // 清除串口发送DMA中断标志
        RS485_DIR = 0;           // 发送完成，P25低电平使能接收
        s_uart3_tx_ready = 1;
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
    if (DMA_UART3_CheckRxFlag()) // 判断串口接收DMA中断
    {
        DMA_UART3_ClearRxFlag();
        s_uart3_rx_activity = 1u;
        UART3_PublishRx(UART3_RX_DMA_BUFFER_SIZE);
        //基本上不考虑 这种情况 基本都可以用总线空闲的包来触发
    }
    //<<AICUBE_USER_UART3_ISR_CODE3_END>>
}

void UART3_ISR(void) interrupt UART3_VECTOR
{
    uint8_t rx_count;

    if (UART3_CheckTxFlag())
    {
        UART3_ClearTxFlag();
    }
    if (UART3_CheckTimeoutFlag())
    {
        UART_DEBUG = 0;
        UART3_ClearTimeoutFlag();
        rx_count = UART3_GetRxCount();
        if (rx_count > 0u)
        {
            s_uart3_rx_activity = 1u;
            UART3_PublishRx(rx_count);
        }
        else if ((s_uart3_rx_activity != 0u) && (s_uart3_timeout_retry == 0u))
        {
            s_uart3_timeout_retry = 1u;
        }
        else
        {
            UART3_RestartRx();
        }
        UART_DEBUG = 1;
    }

    if (UART3_CheckRxFlag())
    {
        s_uart3_rx_activity = 1u;
        UART3_ClearRxFlag();
    }
}

//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
uint8_t xdata *UART_GetRxBuffer(uint8_t *len)
{
    uint8_t ea_state;
    uint8_t read_index;

    if (len == (uint8_t *)0)
    {
        return (uint8_t xdata *)0;
    }

    ea_state = EA;
    DisableGlobalInt();

    if (s_rx_locked)
    {
        EA = ea_state;
        *len = 0;
        return (uint8_t xdata *)0;
    }

    if ((s_rx_pending_count == 0u) || (s_rx_publish_index == UART3_INVALID_SLOT))
    {
        EA = ea_state;
        *len = 0;
        return (uint8_t xdata *)0;
    }

    read_index = s_rx_publish_index;
    s_rx_locked = 1;
    s_rx_read_index = read_index;
    *len = s_uart3_rx_len[read_index];
    s_rx_pending_count--;
    if (s_rx_pending_count == 0u)
    {
        s_rx_publish_index = UART3_INVALID_SLOT;
    }
    else
    {
        s_rx_publish_index = UART3_FindNextPendingSlot((uint8_t)(read_index + 1u));
    }
    EA = ea_state;
    return (uint8_t xdata *)s_uart3_rx_dma_buffer[read_index];
}

void UART_ReleaseRxBuffer(void)
{
    uint8_t ea_state;

    ea_state = EA;
    DisableGlobalInt();
    if (!s_rx_locked || (s_rx_read_index >= UART3_RX_SLOT_COUNT))
    {
        EA = ea_state;
        return;
    }

    s_uart3_rx_len[s_rx_read_index] = 0;
    // memset((void *)s_uart3_rx_dma_buffer[s_rx_read_index], 0, UART3_RX_DMA_BUFFER_SIZE);
    s_rx_locked = 0;
    s_rx_read_index = UART3_INVALID_SLOT;
    EA = ea_state;
}

//! 在改变dma数组的时候注意修改len的值
uint8_t UART_DMA_Send(const uint8_t *pData, uint8_t len)
{
    if ((pData == (const uint8_t *)0) || (len == 0))
    {
        return 0;
    }

    if (len > UART3_TX_DMA_BUFFER_SIZE)
    {
        len = UART3_TX_DMA_BUFFER_SIZE;
    }

    if (!UART3_WaitLockTx())
    {
        return 0;
    }

    /* DMA 发送长度已单独配置，无需每次清空整块缓冲。 */
    memcpy(pu8UR3DMATxBuffer, pData, len);
    DMA_UART3_SetTxAmount((uint16_t)(len - 1u));
    DMA_UART3_ClearTxFlag();
    RS485_DIR = 1; // P25高电平，使能RS485发送
    DMA_UART3_TriggerTx();

    return 1;
}
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>
