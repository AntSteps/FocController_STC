//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: uart.h
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#ifndef __UART_H__
#define __UART_H__


//<<AICUBE_USER_DEFINE_BEGIN>>
void UART_DMA_Send(uint8_t *pData, uint8_t len);
void UART_DMA_Receive(uint8_t *pData, uint8_t len);
//<<AICUBE_USER_DEFINE_END>>


void UART3_Init(void);

extern uint8_t xdata pu8UR3DMATxBuffer[22];
extern uint8_t xdata pu8UR3DMARxBuffer[18];


//<<AICUBE_USER_EXTERNAL_DECLARE_BEGIN>>
extern uint8_t rxd_buf[18 * 2];    //串口接收缓冲区数组
//<<AICUBE_USER_EXTERNAL_DECLARE_END>>


#endif
