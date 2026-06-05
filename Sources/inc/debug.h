#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "config.h"

/* 调试模式开关: 1=输出调试数据包, 0=正常协议模式 */
#define DEBUG_MODE_ENABLE   0

#define DBG_INTERVAL_MS     5u

#if DEBUG_MODE_ENABLE
void Debug_Init(void);
void Debug_Poll(void);
#endif

#endif
