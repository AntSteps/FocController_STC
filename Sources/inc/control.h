#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "def.h"

/*==========================================
 * 电机控制库 - 包含速度环、滤波器、前馈
 *=========================================*/

/* 速度观测器选择：0=角度差分+LPF，1=PLL */
#define SPEED_OBSERVER_METHOD   1

/*------------------------------------------
 * 一阶低通滤波器
 *------------------------------------------*/
typedef struct {
    float alpha;        // 滤波系数 (0~1)，越小滤波越强
    float output;       // 滤波输出
    uint8 init_done;    // 初始化标志
} LowPassFilter_t;

// 初始化滤波器，time_constant_ms为时间常数，sample_freq_hz为采样频率
void LPF_Init(LowPassFilter_t *lpf, float time_constant_ms, float sample_freq_hz);
// 重置滤波器
void LPF_Reset(LowPassFilter_t *lpf);
// 更新滤波器，返回滤波后的值
float LPF_Update(LowPassFilter_t *lpf, float input);


/*------------------------------------------
 * 二自由度 PI 控制器（带抗积分饱和）
 * 比例项使用 β*ref - feedback，积分项使用 ref - feedback
 * beta=1 时退化为标准PI
 *------------------------------------------*/
typedef struct {
    // 参数
    float kp;           // 比例系数
    float ki;           // 积分系数
    float beta;         // 二自由度系数 (0~1)，1为标准PI
    float out_max;      // 输出上限
    float out_min;      // 输出下限

    // 状态
    float integral;     // 积分累加值
    float output;       // 上次输出
} PI_Controller_t;

// 初始化PI控制器（beta: 二自由度系数，1为标准PI）
void PI_Init(PI_Controller_t *pi, float kp, float ki, float beta, float out_max, float out_min);
// 重置PI控制器（清除积分）
void PI_Reset(PI_Controller_t *pi);
// 更新PI控制器，传入参考值和反馈值，返回输出
float PI_Update(PI_Controller_t *pi, float ref, float feedback);


/*------------------------------------------
 * PLL速度观测器
 * 相位检测器 → PI环路滤波器 → 积分器
 * 天然滤波，低速性能好，噪声抑制强
 *------------------------------------------*/
typedef struct {
    /* 参数 */
    float kp;               /* 环路比例增益 */
    float ki;               /* 环路积分增益 */

    /* 状态 */
    float est_angle;        /* 估计角度 */
    float est_speed;        /* 估计速度（rad/s） */
    float integral;         /* 积分项（PI环路积分器） */

    /* Tustin历史量 */
    float prev_angle_err;   /* 上次角度误差 */
    float prev_speed;       /* 上次估计速度 */
} PLL_SpeedObserver_t;

// 初始化PLL观测器，bandwidth_hz为环路带宽(Hz)，zeta为阻尼比
void PLL_Init(PLL_SpeedObserver_t *pll, float bandwidth_hz, float zeta, float initial_angle);
// 重置PLL观测器
void PLL_Reset(PLL_SpeedObserver_t *pll, float initial_angle);
// 更新PLL观测器，传入测量机械角度(rad)，返回估计速度(rad/s)
float PLL_Update(PLL_SpeedObserver_t *pll, float measured_angle, float sample_freq_hz);
// 获取PLL估计速度
float PLL_GetSpeed(PLL_SpeedObserver_t *pll);


/*------------------------------------------
 * 前馈控制器接口（预留扩展）
 *------------------------------------------*/
typedef struct {
    float velocity_gain;        /* 速度前馈增益 */
    float acceleration_gain;    /* 加速度前馈增益（预留） */
    float friction_compensation;/* 摩擦补偿（预留） */
} FeedForward_Params_t;

void FeedForward_Init(FeedForward_Params_t *ff, float velocity_gain);
float FeedForward_Calculate(FeedForward_Params_t *ff, float target_velocity);


/*------------------------------------------
 * 速度环控制器参数结构体
 * 所有配置参数封装在一起
 *------------------------------------------*/
typedef struct {
    /* PI控制器参数 */
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float beta;                 /* 二自由度系数 (0~1)，1为标准PI */
    float out_max;              /* 输出上限 */
    float out_min;              /* 输出下限 */

    /* 滤波器参数 */
    float lpf_time_constant_ms; /* 速度反馈滤波时间常数(ms) */
    float cmd_lpf_time_constant_ms; /* 命令整形滤波时间常数(ms)，0表示不滤波 */

    /* 前馈参数 */
    FeedForward_Params_t ff;
    uint8 enable_feedforward;   /* 前馈使能标志 */

    /* PLL观测器参数 */
    float pll_bandwidth_hz;     /* PLL环路带宽(Hz) */
    float pll_zeta;             /* PLL阻尼比 */

    /* 系统参数 */
    float sample_freq_hz;       /* 速度环采样频率(Hz) */
} SpeedController_Params_t;


/*------------------------------------------
 * 速度环控制器状态结构体
 *------------------------------------------*/
typedef struct {
    /* 子模块 */
    PI_Controller_t pi;
    LowPassFilter_t speed_lpf;
    LowPassFilter_t cmd_lpf;    /* 命令整形滤波器 */

    /* 配置参数（指针引用，生命周期由调用者保证） */
    SpeedController_Params_t *params;

    /* 运行状态 */
    float target_speed;         /* 目标速度（命令整形后） */
    float raw_target_speed;     /* 原始目标速度（整形前） */
    float filtered_speed;       /* 滤波后速度 */
    float last_angle;           /* 上次角度（用于差分） */
    uint8 angle_initialized;    /* 角度初始化标志 */
    float pi_output;            /* PI输出 */
    float ff_output;            /* 前馈输出 */
    float total_output;         /* 总输出（PI + 前馈） */

#if SPEED_OBSERVER_METHOD == 1
    PLL_SpeedObserver_t pll;    /* PLL速度观测器 */
#endif
} SpeedController_t;

/* 全局速度环控制器（单例） */
extern SpeedController_t g_speed_ctrl;

/* 初始化速度环 - 通过参数结构体（指针需保证生命周期全程有效） */
void SpeedController_Init(SpeedController_Params_t *params);
// 设置目标速度
void SpeedController_SetTarget(float target_speed);
// 获取目标速度
float SpeedController_GetTarget(void);
// 获取滤波后的速度
float SpeedController_GetFilteredSpeed(void);
// 获取控制器总输出
float SpeedController_GetOutput(void);
// 更新速度环（传入当前机械角度，单位rad），返回输出（q轴电压/电流参考）
float SpeedController_Update(float mechanical_angle);
// 重置速度环
void SpeedController_Reset(void);
// 运行时刷新参数，保留目标速度
void SpeedController_RefreshParams(SpeedController_Params_t *params);
// 设置前馈增益
void SpeedController_SetFeedForwardGain(float velocity_gain);
// 使能/禁用前馈
void SpeedController_EnableFeedForward(uint8 enable);


#endif /* __CONTROL_H__ */
