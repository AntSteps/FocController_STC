//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: timer.h
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#ifndef __TIMER_H__
#define __TIMER_H__

#include "def.h"

void TIMER0_Init(void);
void TIMER3_Init(void);
uint32_t Timer_GetTickMs(void);

#endif
