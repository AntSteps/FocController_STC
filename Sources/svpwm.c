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
static float data s_half_period = 0.0f;
/* 常量定义 */
#define SQRT3_2 0.86602540378f /* sqrt(3)/2 */
#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define ONE_OVER_2PI 0.15915494309f /* 1/(2*PI) */

//<<AICUBE_USER_GLOBAL_DEFINE_END>>

////////////////////////////////////////
// SVPWM初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void SVPWM_Init(void)
{
    /* 缓存周期值，PWM重载值为399，所以周期是399+1=400 */
    s_pwm_period = 399;
    s_half_period = (float)(s_pwm_period + 1) * 0.5f;
    /* 确保PWM通道已初始化并运行 */
    /* 这里假设PWMA_Init()已在SYS_Init()中调用 */
}

////////////////////////////////////////
// 核心SVPWM更新 - 纯数学运算
// 入口参数: v_alpha: 静止坐标系alpha轴电压
//           v_beta:  静止坐标系beta轴电压
// 函数返回: 无
////////////////////////////////////////
// 常量预定义 (保持不变)
#define SQRT3_2 0.86602540378f
#define TWO_PI 6.28318530718f




////////////////////////////////////////
// SVPWM更新 - 高性能压榨版
////////////////////////////////////////
void SVPWM_Update(float angle_el, float mag)
{
    // 不要加 data 关键字，让编译器全部分配进寄存器
    float sin_theta, cos_theta;
    float v_alpha, v_beta;
    float v_a, v_b, v_c;
    float v_min, v_max, v_offset;
    float duty_a, duty_b, duty_c;
    float temp_alpha, temp_beta; // 用于复用计算结果

    // 1. 简化的角度归一化 (直接循环减，避免 float-int-float 转换的开销)
    // 假设上层给的角度通常不会偏离 0~2PI 太远
    while (angle_el < 0.0f)
        angle_el += TWO_PI;
    while (angle_el >= TWO_PI)
        angle_el -= TWO_PI;

    // 2. 强制使用单精度数学函数 (极速调用硬件 TFPU)
    sin_theta = sin(angle_el);
    cos_theta = cos(angle_el);

    // 3. 反Park变换
    v_alpha = mag * cos_theta;
    v_beta = mag * sin_theta;

    // 4. 反Clarke变换 (提取公共子表达式，省去两次乘法)
    v_a = v_alpha;
    temp_alpha = -0.5f * v_alpha;
    temp_beta = SQRT3_2 * v_beta;
    v_b = temp_alpha + temp_beta;
    v_c = temp_alpha - temp_beta;

    // 5. 零序注入 (Min-Max 三目运算符压榨)
    v_max = v_a > v_b ? (v_a > v_c ? v_a : v_c) : (v_b > v_c ? v_b : v_c);
    v_min = v_a < v_b ? (v_a < v_c ? v_a : v_c) : (v_b < v_c ? v_b : v_c);
    v_offset = -0.5f * (v_max + v_min);

    // 6. 占空比计算 (减少乘法，提取半周期变量)
    // 公式变型： duty = half_period + half_period * (v + offset)
    v_offset += 1.0f; // 巧妙融合常数 1.0f
    duty_a = s_half_period * (v_a + v_offset);
    duty_b = s_half_period * (v_b + v_offset);
    duty_c = s_half_period * (v_c + v_offset);

    // 7. 限幅保护 (三目运算符)
    duty_a = duty_a < 0.0f ? 0.0f : (duty_a > s_pwm_period ? s_pwm_period : duty_a);
    duty_b = duty_b < 0.0f ? 0.0f : (duty_b > s_pwm_period ? s_pwm_period : duty_b);
    duty_c = duty_c < 0.0f ? 0.0f : (duty_c > s_pwm_period ? s_pwm_period : duty_c);

    // 8. 直接写入 PWM 寄存器 (强烈建议替换掉原来的函数调用)

    PWM_UpdateDuty(PWMA_CH1, (uint16_t)duty_a);

    PWM_UpdateDuty(PWMA_CH2, (uint16_t)duty_b);

    PWM_UpdateDuty(PWMA_CH3, (uint16_t)duty_c);
}

void SVPWM_UpdateQAxis(float angle_el, float mag)
{
    float sin_theta, cos_theta;
    float v_alpha, v_beta;
    float v_a, v_b, v_c;
    float v_min, v_max, v_offset;
    float duty_a, duty_b, duty_c;
    float temp_alpha, temp_beta;

    sin_theta = sin(angle_el);
    cos_theta = cos(angle_el);

    v_alpha = -mag * sin_theta;
    v_beta = mag * cos_theta;

    v_a = v_alpha;
    temp_alpha = -0.5f * v_alpha;
    temp_beta = SQRT3_2 * v_beta;
    v_b = temp_alpha + temp_beta;
    v_c = temp_alpha - temp_beta;

    v_max = v_a > v_b ? (v_a > v_c ? v_a : v_c) : (v_b > v_c ? v_b : v_c);
    v_min = v_a < v_b ? (v_a < v_c ? v_a : v_c) : (v_b < v_c ? v_b : v_c);
    v_offset = -0.5f * (v_max + v_min);

    v_offset += 1.0f;
    duty_a = s_half_period * (v_a + v_offset);
    duty_b = s_half_period * (v_b + v_offset);
    duty_c = s_half_period * (v_c + v_offset);

    duty_a = duty_a < 0.0f ? 0.0f : (duty_a > s_pwm_period ? s_pwm_period : duty_a);
    duty_b = duty_b < 0.0f ? 0.0f : (duty_b > s_pwm_period ? s_pwm_period : duty_b);
    duty_c = duty_c < 0.0f ? 0.0f : (duty_c > s_pwm_period ? s_pwm_period : duty_c);

    PWM_UpdateDuty(PWMA_CH1, (uint16_t)duty_a);
    PWM_UpdateDuty(PWMA_CH2, (uint16_t)duty_b);
    PWM_UpdateDuty(PWMA_CH3, (uint16_t)duty_c);
}

//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>
