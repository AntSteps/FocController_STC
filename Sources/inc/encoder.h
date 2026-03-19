#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "config.h"
#include "spi.h"

sbit ENCODER_CS = P3^5;

#define ENCODER_MAG_FIELD_NORMAL      0u
#define ENCODER_MAG_FIELD_TOO_STRONG  1u
#define ENCODER_MAG_FIELD_TOO_WEAK    2u
#define ENCODER_MAG_FIELD_INVALID     3u

void Encoder_Init(void);
void Encoder_Update();
float Encoder_UpdateAndGetAngle(void);

/* 返回机械角，单位: rad，范围约 [0, 2PI) */
float Encoder_Get_Angle(void);
uint8_t Encoder_Get_DataGood(void);

/* 返回 Mg[3:0] 原始状态 */
uint8_t Encoder_Get_MagStatus(void);

/* 返回 Mg[1:0] 磁场状态，见 ENCODER_MAG_FIELD_* */
uint8_t Encoder_Get_MagFieldStatus(void);

/* 返回 Mg[2]，1 表示旋钮被按压 */
uint8_t Encoder_Get_PressStatus(void);

/* 返回 Mg[3]，1 表示超速 */
uint8_t Encoder_Get_SpeedStatus(void);

#endif // __ENCODER_H__