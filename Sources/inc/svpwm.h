#ifndef __SVPWM_H__
#define __SVPWM_H__



void SVPWM_Init(void);
void SVPWM_Update(float angle_el, float mag);
void SVPWM_UpdateQAxis(float angle_el, float mag);
void SVPWM_UpdateCore(float v_alpha, float v_beta);


#endif
