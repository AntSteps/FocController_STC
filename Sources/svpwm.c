//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息
// 文件名称: svpwm.c
// 文件描述: SVPWM空间矢量脉宽调制驱动
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-13) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#include "config.h"


//<<AICUBE_USER_INCLUDE_BEGIN>>
#include "svpwm.h"
//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
// 在此添加用户全局变量定义、用户宏定义以及函数声明

/* 缓存PWM周期值，避免重复读取寄存器 */
static uint16_t s_pwm_period = 0;

/* 常量定义 */
#define SQRT3_2     0.86602540378f  /* sqrt(3)/2 */
#define PI          3.14159265359f
#define TWO_PI      6.28318530718f
#define ONE_OVER_2PI 0.15915494309f /* 1/(2*PI) */

//<<AICUBE_USER_GLOBAL_DEFINE_END>>


////////////////////////////////////////
// 角度归一化到 [-PI, PI]
// 入口参数: angle: 输入角度 [rad]
// 函数返回: 归一化后的角度 [rad]
////////////////////////////////////////
static float angle_normalize(float angle)
{
    float turns;
    int32_t turns_int;
    float wrapped_angle;

    /* 快速角度归一化: angle - 2*PI * round(angle / (2*PI)) */
    turns = angle * ONE_OVER_2PI;
    turns_int = (int32_t)(turns > 0.0f ? turns + 0.5f : turns - 0.5f);
    wrapped_angle = angle - TWO_PI * (float)turns_int;

    return wrapped_angle;
}

////////////////////////////////////////
// SVPWM初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SVPWM_Init(void)
{
    /* 缓存周期值，PWM重载值为399，所以周期是399+1=400 */
    s_pwm_period = 399;

    /* 确保PWM通道已初始化并运行 */
    /* 这里假设PWMA_Init()已在SYS_Init()中调用 */
}

////////////////////////////////////////
// 核心SVPWM更新 - 纯数学运算
// 入口参数: v_alpha: 静止坐标系alpha轴电压
//           v_beta:  静止坐标系beta轴电压
// 函数返回: 无
////////////////////////////////////////
void SVPWM_UpdateCore(float v_alpha, float v_beta)
{
    float v_a;
    float v_b;
    float v_c;
    float v_min;
    float v_max;
    float v_offset;
    float half_period;
    float duty_a;
    float duty_b;
    float duty_c;
    float period_f;
    uint16_t duty_u16;

    /* 1. 反Clarke变换 (alpha/beta -> abc) */
    v_a = v_alpha;
    v_b = -0.5f * v_alpha + SQRT3_2 * v_beta;
    v_c = -0.5f * v_alpha - SQRT3_2 * v_beta;

    /* 2. 零序注入（Min-Max Method） */
    v_min = v_a;
    v_max = v_a;

    if (v_b < v_min)
        v_min = v_b;
    if (v_c < v_min)
        v_min = v_c;
    if (v_b > v_max)
        v_max = v_b;
    if (v_c > v_max)
        v_max = v_c;

    v_offset = -0.5f * (v_max + v_min);

    /* 3. 转换为占空比并限幅 */
    half_period = (float)s_pwm_period * 0.5f;

    duty_a = half_period * (1.0f + v_a + v_offset);
    duty_b = half_period * (1.0f + v_b + v_offset);
    duty_c = half_period * (1.0f + v_c + v_offset);

    period_f = (float)s_pwm_period;

    if (duty_a < 0.0f)
        duty_a = 0.0f;
    else if (duty_a > period_f)
        duty_a = period_f;

    if (duty_b < 0.0f)
        duty_b = 0.0f;
    else if (duty_b > period_f)
        duty_b = period_f;

    if (duty_c < 0.0f)
        duty_c = 0.0f;
    else if (duty_c > period_f)
        duty_c = period_f;

    /* 4. 写入PWM寄存器 */
    duty_u16 = (uint16_t)duty_a;
    PWM_UpdateDuty(PWMA_CH1, duty_u16);

    duty_u16 = (uint16_t)duty_b;
    PWM_UpdateDuty(PWMA_CH2, duty_u16);

    duty_u16 = (uint16_t)duty_c;
    PWM_UpdateDuty(PWMA_CH3, duty_u16);
}

////////////////////////////////////////
// SVPWM更新 - 角度+幅值输入
// 入口参数: angle_el: 电角度 [rad]
//           mag: 调制比 [0, 1.1547]，建议限制在[0, 1]
// 函数返回: 无
//
// 使用math库的sin/cos函数计算
////////////////////////////////////////
void SVPWM_Update(float angle_el, float mag)
{
    float normalized_angle;
    float sin_theta;
    float cos_theta;
    float v_alpha;
    float v_beta;

    /* 角度归一化到 [-PI, PI] 范围 */
    normalized_angle = angle_normalize(angle_el);

    /* 使用math库的sin和cos函数（替代CORDIC硬件） */
    sin_theta = sin(normalized_angle);
    cos_theta = cos(normalized_angle);

    /* 反Park变换: (d=mag, q=0) -> (v_alpha, v_beta) */
    v_alpha = mag * cos_theta;
    v_beta = mag * sin_theta;

    /* 调用核心SVPWM */
    SVPWM_UpdateCore(v_alpha, v_beta);
}



//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


