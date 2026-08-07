#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "def.h"

/*==========================================
 * 电机控制库 - 包含速度环、滤波器、前馈
 *=========================================*/

/* 速度观测器选择：0=角度差分+LPF，1=PLL */
#define SPEED_OBSERVER_METHOD   1

/* 高频观测器与低频速度环的更新频率。 */
#define SPEED_OBSERVER_SAMPLE_FREQ_HZ  5000.0f
#define SPEED_CTRL_DECIMATION          17u
#define SPEED_CTRL_SAMPLE_FREQ_HZ      (SPEED_OBSERVER_SAMPLE_FREQ_HZ / (float)SPEED_CTRL_DECIMATION)

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
// 更新PI控制器，比例项和积分项可使用不同的反馈速度
float PI_Update(PI_Controller_t *pi, float ref, float p_feedback, float i_feedback);


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
 * 前馈参数结构体（预留扩展）
 *------------------------------------------*/
typedef struct {
    float acceleration_gain;    /* 加速度前馈增益 */
    float friction_compensation;/* 摩擦补偿系数 */
    float R;                    /* 相电阻 (Ohm) */
    float Ke;                   /* 反电动势系数 (V/(rad/s)) */
} FeedForward_Params_t;


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
    float lpf_time_constant_ms; /* 速度趋势滤波时间常数(ms) */
    float cmd_lpf_time_constant_ms; /* 命令整形滤波时间常数(ms)，0表示不滤波 */

    /* 前馈参数 */
    FeedForward_Params_t ff;

    /* PLL观测器参数 */
    float pll_bandwidth_hz;     /* PLL环路带宽(Hz) */
    float pll_zeta;             /* PLL阻尼比 */

    /* 非线性残差旁路参数 */
    float residual_rho;         /* 小残差保留系数(0~1) */
    float residual_deadband;    /* 残差软死区(rad/s) */
} SpeedController_Params_t;


/*------------------------------------------
 * 速度环控制器状态结构体
 *------------------------------------------*/
typedef struct {
    /* 子模块 */
    PI_Controller_t pi;
    LowPassFilter_t observer_lpf; /* 差分测速模式的速度滤波器 */
    LowPassFilter_t trend_lpf;    /* 速度环趋势滤波器 */
    LowPassFilter_t cmd_lpf;    /* 命令整形滤波器 */

    /* 配置参数（指针引用，生命周期由调用者保证） */
    SpeedController_Params_t *params;

    /* 运行状态 */
    float target_speed;         /* 目标速度（命令整形后） */
    float raw_target_speed;     /* 原始目标速度（整形前） */
    float fast_speed;           /* PLL降采样后的快速速度 */
    float trend_speed;          /* 低通后的趋势速度 */
    float filtered_speed;       /* 最终进入比例速度环的等效反馈速度 */
    float last_angle;           /* 上次角度（用于差分） */
    uint8 angle_initialized;    /* 角度初始化标志 */
    float observer_speed_sum;   /* 高频观测速度的降采样前累加值 */
    uint8 observer_sample_count;

    /* 电流环各项输出 */
    float pi_output;            /* PI输出 */
    float accel_ff;             /* 加速度前馈 */
    float load_ff;              /* 负载前馈（上层提供） */
    float friction_ff;          /* 摩擦前馈 */
    float i_output;             /* 总电流参考 = pi + accel_ff + load_ff + friction_ff */
    float total_output;         /* 电压输出 = i_output * R + Ke * ω */

    /* 加速度计算用 */
    float prev_target_speed;    /* 上次目标速度（用于差分） */

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
// 高频更新速度观测器（传入校正机械角，单位rad），返回瞬时估计速度
float SpeedController_UpdateObserver(float mechanical_angle);
// 累计满一个速度环周期时返回平均速度
BOOL SpeedController_TryGetAveragedSpeed(float *averaged_speed);
// 低频更新速度PI（传入降采样前平均后的速度），返回q轴电压
float SpeedController_UpdateFromSpeed(float measured_speed);
// 重置速度环
void SpeedController_Reset(void);
// 运行时刷新参数，保留目标速度
void SpeedController_RefreshParams(SpeedController_Params_t *params);
// 设置负载前馈（上层提供）
void SpeedController_SetLoadFF(float load_ff);


#endif /* __CONTROL_H__ */
