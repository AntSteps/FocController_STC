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
void Write_RXD_Buffer(uint8_t *dat, uint8_t len);
//<<AICUBE_USER_DEFINE_END>>


void UART3_Init(void);

extern uint8_t xdata pu8UR3DMATxBuffer[22];
extern uint8_t xdata pu8UR3DMARxBuffer[18];


//<<AICUBE_USER_EXTERNAL_DECLARE_BEGIN>>
extern uint8_t rxd_buf[18 * 5];    //串口接收缓冲区数组
extern uint8_t write_ptr;          //循环数组的当前索引 // FIXME: uint8_t注意下
extern uint8_t read_ptr;           //循环数组的读取索引
//<<AICUBE_USER_EXTERNAL_DECLARE_END>>


#endif
