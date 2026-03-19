#include "control.h"

#define PI 3.141592654f
#define TWO_PI 6.283185307f

/*==========================================
 * 全局单例 - 速度环控制器
 *=========================================*/
SpeedController_t g_speed_ctrl;


/*==========================================
 * 低通滤波器实现
 *=========================================*/
void LPF_Init(LowPassFilter_t *lpf, float time_constant_ms, float sample_freq_hz)
{
    /* alpha = 1 / (1 + tau * fs)，其中 tau 为时间常数(秒) */
    float tau = time_constant_ms / 1000.0f;
    lpf->alpha = 1.0f / (1.0f + tau * sample_freq_hz);
    if (lpf->alpha > 1.0f) lpf->alpha = 1.0f;
    if (lpf->alpha < 0.0f) lpf->alpha = 0.0f;

    lpf->output = 0.0f;
    lpf->init_done = 0;
}

void LPF_Reset(LowPassFilter_t *lpf)
{
    lpf->output = 0.0f;
    lpf->init_done = 0;
}

float LPF_Update(LowPassFilter_t *lpf, float input)
{
    if (!lpf->init_done) {
        /* 首次初始化，直接赋值 */
        lpf->output = input;
        lpf->init_done = 1;
    } else {
        /* 一阶低通滤波: y[n] = y[n-1] + alpha * (x[n] - y[n-1]) */
        lpf->output += lpf->alpha * (input - lpf->output);
    }
    return lpf->output;
}


/*==========================================
 * PI 控制器实现
 *=========================================*/
void PI_Init(PI_Controller_t *pi, float kp, float ki, float out_max, float out_min)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_max = out_max;
    pi->out_min = out_min;
    pi->integral = 0.0f;
    pi->output = 0.0f;
}

void PI_Reset(PI_Controller_t *pi)
{
    pi->integral = 0.0f;
    pi->output = 0.0f;
}

float PI_Update(PI_Controller_t *pi, float error)
{
    float p_term, i_term, output;

    /* 比例项 */
    p_term = pi->kp * error;

    /* 积分项累加 */
    pi->integral += error;

    /* 计算输出（未限幅） */
    i_term = pi->ki * pi->integral;
    output = p_term + i_term;

    /* 输出限幅 + 抗积分饱和 */
    if (output > pi->out_max) {
        output = pi->out_max;
        /* 如果积分导致饱和，回退积分 */
        if (error > 0 && pi->ki > 0) {
            pi->integral -= error;
        }
    } else if (output < pi->out_min) {
        output = pi->out_min;
        /* 如果积分导致饱和，回退积分 */
        if (error < 0 && pi->ki > 0) {
            pi->integral -= error;
        }
    }

    pi->output = output;
    return output;
}


/*==========================================
 * 前馈控制器实现
 *=========================================*/
void FeedForward_Init(FeedForward_Params_t *ff, float velocity_gain)
{
    ff->velocity_gain = velocity_gain;
    ff->acceleration_gain = 0.0f;
    ff->friction_compensation = 0.0f;
}

float FeedForward_Calculate(FeedForward_Params_t *ff, float target_velocity)
{
    return ff->velocity_gain * target_velocity;
}


/*==========================================
 * 速度环控制器实现
 *=========================================*/
void SpeedController_Init(SpeedController_Params_t *params)
{
    /* 保存参数指针引用 */
    g_speed_ctrl.params = params;

    /* 初始化子模块 */
    PI_Init(&g_speed_ctrl.pi, params->kp, params->ki, params->out_max, params->out_min);
    LPF_Init(&g_speed_ctrl.speed_lpf, params->lpf_time_constant_ms, params->sample_freq_hz);

    /* 初始化状态 */
    g_speed_ctrl.target_speed = 0.0f;
    g_speed_ctrl.filtered_speed = 0.0f;
    g_speed_ctrl.last_angle = 0.0f;
    g_speed_ctrl.angle_initialized = 0;
    g_speed_ctrl.pi_output = 0.0f;
    g_speed_ctrl.ff_output = 0.0f;
    g_speed_ctrl.total_output = 0.0f;
}

void SpeedController_SetTarget(float target_speed)
{
    g_speed_ctrl.target_speed = target_speed;
}

float SpeedController_GetTarget(void)
{
    return g_speed_ctrl.target_speed;
}

float SpeedController_Update(float mechanical_angle)
{
    float delta_angle, raw_speed, speed_error;

    if (!g_speed_ctrl.angle_initialized) {
        /* 首次运行，记录角度，不计算速度 */
        g_speed_ctrl.last_angle = mechanical_angle;
        g_speed_ctrl.angle_initialized = 1;
        return g_speed_ctrl.total_output;  /* 返回上次输出（初始为0） */
    }

    /* 计算角度差分（处理跨圈情况） */
    delta_angle = mechanical_angle - g_speed_ctrl.last_angle;

    /* 处理角度跳变（2PI 边界） */
    if (delta_angle > PI) {
        delta_angle -= TWO_PI;
    } else if (delta_angle < -PI) {
        delta_angle += TWO_PI;
    }

    g_speed_ctrl.last_angle = mechanical_angle;

    /* 计算原始速度 (rad/s) */
    raw_speed = delta_angle * g_speed_ctrl.params->sample_freq_hz;

    /* 滤波 */
    g_speed_ctrl.filtered_speed = LPF_Update(&g_speed_ctrl.speed_lpf, raw_speed);

    /* 速度误差 */
    speed_error = g_speed_ctrl.target_speed - g_speed_ctrl.filtered_speed;

    /* PI 计算 */
    g_speed_ctrl.pi_output = PI_Update(&g_speed_ctrl.pi, speed_error);

    /* 前馈计算（如果使能） */
    if (g_speed_ctrl.params->enable_feedforward) {
        g_speed_ctrl.ff_output = FeedForward_Calculate(&g_speed_ctrl.params->ff,
                                                       g_speed_ctrl.target_speed);
    } else {
        g_speed_ctrl.ff_output = 0.0f;
    }

    /* 总输出 = PI输出 + 前馈输出 */
    g_speed_ctrl.total_output = g_speed_ctrl.pi_output + g_speed_ctrl.ff_output;

    /* 最终限幅（确保在安全范围） */
    if (g_speed_ctrl.total_output > g_speed_ctrl.pi.out_max) {
        g_speed_ctrl.total_output = g_speed_ctrl.pi.out_max;
    } else if (g_speed_ctrl.total_output < g_speed_ctrl.pi.out_min) {
        g_speed_ctrl.total_output = g_speed_ctrl.pi.out_min;
    }

    return g_speed_ctrl.total_output;
}

void SpeedController_Reset(void)
{
    PI_Reset(&g_speed_ctrl.pi);
    LPF_Reset(&g_speed_ctrl.speed_lpf);
    g_speed_ctrl.angle_initialized = 0;
    g_speed_ctrl.pi_output = 0.0f;
    g_speed_ctrl.ff_output = 0.0f;
    g_speed_ctrl.total_output = 0.0f;
}

void SpeedController_SetFeedForwardGain(float velocity_gain)
{
    g_speed_ctrl.params->ff.velocity_gain = velocity_gain;
}

void SpeedController_EnableFeedForward(uint8 enable)
{
    g_speed_ctrl.params->enable_feedforward = enable;
}
