//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息
// 文件名称: svpwm.h
// 文件描述: SVPWM空间矢量脉宽调制驱动
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-13) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#ifndef __SVPWM_H__
#define __SVPWM_H__


//<<AICUBE_USER_DEFINE_BEGIN>>
// 在此添加用户宏定义
//<<AICUBE_USER_DEFINE_END>>


void SVPWM_Init(void);
void SVPWM_Update(float angle_el, float mag);
void SVPWM_UpdateQAxis(float angle_el, float mag);
void SVPWM_UpdateCore(float v_alpha, float v_beta);



//<<AICUBE_USER_EXTERNAL_DECLARE_BEGIN>>
// 在此添加用户外部函数和外部变量声明
//<<AICUBE_USER_EXTERNAL_DECLARE_END>>


#endif
