#ifndef __UART_H__
#define __UART_H__


#define UART3_TX_DMA_BUFFER_SIZE    128
#define UART3_RX_DMA_BUFFER_SIZE    64


//<<AICUBE_USER_DEFINE_BEGIN>>
uint8_t xdata *UART_GetRxBuffer(uint8_t *len);
void UART_ReleaseRxBuffer(void);
uint8_t xdata *UART_PeekActiveRxBuffer(uint8_t *slot_index, uint8_t *len);
BOOL UART_PublishActiveRxBuffer(uint8_t slot_index, uint8_t min_len);
uint8_t UART3_TryLockTx(void);
uint8_t UART3_WaitLockTx(void);
uint8_t xdata *UART3_WaitLockTxBuffer(void);
uint8_t UART3_StartLockedTx(uint8_t len);
void UART3_UnlockTx(void);
//<<AICUBE_USER_DEFINE_END>>


void UART3_Init(void);

extern uint8_t xdata pu8UR3DMATxBuffer[UART3_TX_DMA_BUFFER_SIZE];


#endif
