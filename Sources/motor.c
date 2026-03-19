#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"
static uint16_t s_lfsr = 0xACE1u; 
static float s_prbs_out = 0.0f;      // 当前保持的 PRBS 电压

static void PRBS_Update(void) 
{
    // 1. 提取最低位 (LSB)
    uint8_t lsb = s_lfsr & 1; 
    
    // 2. 整体右移一位
    s_lfsr >>= 1;             
    
    // 3. 如果移出的是 1，则与多项式掩码异或 (0xB400 是 16位最大长度多项式)
    if (lsb) {
        s_lfsr ^= 0xB400u;    
    }
    
    // 4. 将 0/1 映射为 -Amplitude / +Amplitude (三目运算符，极快)
    s_prbs_out = lsb ? 0.2 : -0.2;
}


void Motor_Commutation_Update(void)
{
    float elec_angle;

    elec_angle = Motor_Calibration_GetElectricalAngle(Encoder_UpdateAndGetAngle());
    SVPWM_UpdateQAxis(elec_angle, 0.8f);
    printf("%f\n", elec_angle);
}