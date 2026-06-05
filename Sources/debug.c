#include "debug.h"

#if DEBUG_MODE_ENABLE

#include "uart.h"
#include "timer.h"
#include "encoder.h"
#include "motor.h"
#include "control.h"
#include "adc.h"
#include "calibration.h"

sbit RS485_DIR = P2 ^ 5;

/* 帧尾: IEEE 754 NaN (0x7F800000 小端) */
static const uint8_t code s_dbg_tail[4] = {0x00, 0x00, 0x80, 0x7F};

static uint32_t s_dbg_last_ms;

void Debug_Init(void)
{
    s_dbg_last_ms = Timer_GetTickMs();
}

void Debug_Poll(void)
{
    uint32_t now;
    float f_val;

    now = Timer_GetTickMs();
    if ((now - s_dbg_last_ms) < DBG_INTERVAL_MS)
    {
        return;
    }
    s_dbg_last_ms = now;

    /* 等待上次发送完成 */
    UART3_WaitLockTx();

    f_val = SpeedController_GetTarget();
    memcpy(pu8UR3DMATxBuffer,      (const void *)&f_val, 4u);
    f_val = SpeedController_GetFilteredSpeed();
    memcpy(pu8UR3DMATxBuffer + 4u, (const void *)&f_val, 4u);
    f_val = Motor_GetQAxisVoltage();
    memcpy(pu8UR3DMATxBuffer + 8u, (const void *)&f_val, 4u);
    f_val = Motor_Calibration_GetMechanicalAngle(Encoder_Get_Angle());
    memcpy(pu8UR3DMATxBuffer + 12u, (const void *)&f_val, 4u);
    memcpy(pu8UR3DMATxBuffer + 16u, s_dbg_tail, 4u);

    DMA_UART3_SetTxAmount(19u);
    DMA_UART3_ClearTxFlag();
    RS485_DIR = 1;
    DMA_UART3_TriggerTx();
}

#endif
