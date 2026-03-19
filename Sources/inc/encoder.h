#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "config.h"
#include "spi.h"

sbit ENCODER_CS = P3^5;


void Encoder_Init(void);
void Encoder_Update();
float Encoder_UpdateAndGetAngle(void);

/* 返回机械角，单位: rad，范围约 [0, 2PI) */
float Encoder_Get_Angle(void);
uint8_t Encoder_Get_DataGood(void);

#endif // __ENCODER_H__