#ifndef __UART_H__
#define __UART_H__


#define UART3_TX_DMA_BUFFER_SIZE    128
#define UART3_RX_DMA_BUFFER_SIZE    64


//<<AICUBE_USER_DEFINE_BEGIN>>
uint8_t UART_DMA_Send(const uint8_t *pData, uint8_t len);
uint8_t xdata *UART_GetRxBuffer(uint8_t *len);
void UART_ReleaseRxBuffer(void);
//<<AICUBE_USER_DEFINE_END>>


void UART3_Init(void);

extern uint8_t xdata pu8UR3DMATxBuffer[UART3_TX_DMA_BUFFER_SIZE];


#endif
