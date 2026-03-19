//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: uart.h
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#ifndef __UART_H__
#define __UART_H__


#define UART3_TX_DMA_BUFFER_SIZE    22
#define UART3_RX_DMA_BUFFER_SIZE    18
#define UART3_RX_BUFFER_COUNT       2


//<<AICUBE_USER_DEFINE_BEGIN>>
void UART_DMA_Send(uint8_t *pData, uint8_t len);
uint8_t xdata *UART_GetRxBuffer(uint8_t *len);
void UART_ReleaseRxBuffer(void);
//<<AICUBE_USER_DEFINE_END>>


void UART3_Init(void);

extern uint8_t xdata pu8UR3DMATxBuffer[UART3_TX_DMA_BUFFER_SIZE];


//<<AICUBE_USER_EXTERNAL_DECLARE_BEGIN>>
//<<AICUBE_USER_EXTERNAL_DECLARE_END>>


#endif
