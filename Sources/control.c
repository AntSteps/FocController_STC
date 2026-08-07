#include "control.h"

#define PI 3.141592654f
#define TWO_PI 6.283185307f

/*==========================================
 * 全局单例 - 速度环控制器
 *=========================================*/
SpeedController_t g_speed_ctrl;

static float NormalizeAnglePi(float angle)
{
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}


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
void PI_Init(PI_Controller_t *pi, float kp, float ki, float beta, float out_max, float out_min)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->beta = beta;
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

float PI_Update(PI_Controller_t *pi, float ref, float p_feedback, float i_feedback)
{
    float p_term, i_term, output;
    float error_i = ref - i_feedback;             /* 积分项只跟随趋势速度 */
    float error_p = pi->beta * ref - p_feedback;  /* 比例项使用非线性等效反馈 */

    /* 比例项 */
    p_term = pi->kp * error_p;

    /* 积分项累加 */
    pi->integral += error_i;

    /* 计算输出（未限幅） */
    i_term = pi->ki * pi->integral;
    output = p_term + i_term;

    /* 输出限幅 + 抗积分饱和 */
    if (output > pi->out_max) {
        output = pi->out_max;
        /* 如果积分导致饱和，回退积分 */
        if (error_i > 0 && pi->ki > 0) {
            pi->integral -= error_i;
        }
    } else if (output < pi->out_min) {
        output = pi->out_min;
        /* 如果积分导致饱和，回退积分 */
        if (error_i < 0 && pi->ki > 0) {
            pi->integral -= error_i;
        }
    }

    pi->output = output;
    return output;
}


/*==========================================
 * PLL速度观测器实现
 *=========================================*/
#if SPEED_OBSERVER_METHOD == 1
void PLL_Init(PLL_SpeedObserver_t *pll, float bandwidth_hz, float zeta, float initial_angle)
{
    float omega_n = TWO_PI * bandwidth_hz;

    pll->kp = 2.0f * zeta * omega_n;
    pll->ki = omega_n * omega_n;
    pll->est_angle = initial_angle;
    pll->est_speed = 0.0f;
    pll->integral = 0.0f;
    pll->prev_angle_err = 0.0f;
    pll->prev_speed = 0.0f;
}

void PLL_Reset(PLL_SpeedObserver_t *pll, float initial_angle)
{
    pll->est_angle = initial_angle;
    pll->est_speed = 0.0f;
    pll->integral = 0.0f;
    pll->prev_angle_err = 0.0f;
    pll->prev_speed = 0.0f;
}

float PLL_Update(PLL_SpeedObserver_t *pll, float measured_angle, float sample_freq_hz)
{
    float angle_err;
    float dt_half;

    /* 相位检测器：测量角度与估计角度的闭环误差 */
    angle_err = NormalizeAnglePi(measured_angle - pll->est_angle);
    dt_half = 0.5f / sample_freq_hz;

    /* PI环路滤波器（Tustin积分）
     * 梯形积分: integral += (err + prev_err) * dt/2
     * 比欧拉更精确，高带宽下不会发散
     */
    pll->integral += (angle_err + pll->prev_angle_err) * pll->ki * dt_half;
    pll->est_speed = pll->kp * angle_err + pll->integral;
    pll->prev_angle_err = angle_err;

    /* 角度积分器（Tustin积分）
     * est_angle += (speed + prev_speed) * dt/2
     */
    pll->est_angle += (pll->est_speed + pll->prev_speed) * dt_half;
    pll->prev_speed = pll->est_speed;


    pll->est_angle = NormalizeAnglePi(pll->est_angle);

    return pll->est_speed;
}

float PLL_GetSpeed(PLL_SpeedObserver_t *pll)
{
    return pll->est_speed;
}
#endif


/*==========================================
 * 速度环控制器实现
 *=========================================*/
void SpeedController_Init(SpeedController_Params_t *params)
{
    /* 保存参数指针引用 */
    g_speed_ctrl.params = params;

    /* 初始化子模块 */
    PI_Init(&g_speed_ctrl.pi, params->kp, params->ki, params->beta, params->out_max, params->out_min);
    LPF_Init(&g_speed_ctrl.observer_lpf, params->lpf_time_constant_ms, SPEED_OBSERVER_SAMPLE_FREQ_HZ);
    LPF_Init(&g_speed_ctrl.trend_lpf, params->lpf_time_constant_ms, SPEED_CTRL_SAMPLE_FREQ_HZ);
    LPF_Init(&g_speed_ctrl.cmd_lpf, params->cmd_lpf_time_constant_ms, SPEED_CTRL_SAMPLE_FREQ_HZ);

    /* 初始化状态 */
    g_speed_ctrl.target_speed = 0.0f;
    g_speed_ctrl.raw_target_speed = 0.0f;
    g_speed_ctrl.cmd_lpf.output = g_speed_ctrl.target_speed;
    g_speed_ctrl.cmd_lpf.init_done = 1;
    g_speed_ctrl.fast_speed = 0.0f;
    g_speed_ctrl.trend_speed = 0.0f;
    g_speed_ctrl.filtered_speed = 0.0f;
    g_speed_ctrl.last_angle = 0.0f;
    g_speed_ctrl.angle_initialized = 0;
    g_speed_ctrl.observer_speed_sum = 0.0f;
    g_speed_ctrl.observer_sample_count = 0u;
    g_speed_ctrl.pi_output = 0.0f;
    g_speed_ctrl.accel_ff = 0.0f;
    g_speed_ctrl.load_ff = 0.0f;
    g_speed_ctrl.friction_ff = 0.0f;
    g_speed_ctrl.i_output = 0.0f;
    g_speed_ctrl.total_output = 0.0f;
    g_speed_ctrl.prev_target_speed = 0.0f;

#if SPEED_OBSERVER_METHOD == 1
    /* PLL参数在此设置，角度由首次Update时从编码器读取初始化 */
    PLL_Init(&g_speed_ctrl.pll, params->pll_bandwidth_hz, params->pll_zeta, 0.0f);
#endif
}

void SpeedController_SetTarget(float target_speed)
{
    g_speed_ctrl.raw_target_speed = target_speed;
}

float SpeedController_GetTarget(void)
{
    return g_speed_ctrl.target_speed;
}

float SpeedController_GetFilteredSpeed(void)
{
    return g_speed_ctrl.filtered_speed;
}

float SpeedController_GetOutput(void)
{
    return g_speed_ctrl.total_output;
}

float SpeedController_UpdateObserver(float mechanical_angle)
{
    float speed;

    if (!g_speed_ctrl.angle_initialized) {
        /* 首次运行，记录角度，不计算速度 */
        g_speed_ctrl.last_angle = mechanical_angle;
        g_speed_ctrl.angle_initialized = 1;
#if SPEED_OBSERVER_METHOD == 1
        PLL_Reset(&g_speed_ctrl.pll, mechanical_angle);
#endif
        speed = 0.0f;
    }
    else
    {
        #if SPEED_OBSERVER_METHOD == 1
        /* PLL速度观测器：直接传入角度，内部完成差分+滤波 */
        g_speed_ctrl.last_angle = mechanical_angle;
        speed = PLL_Update(&g_speed_ctrl.pll,
                           mechanical_angle,
                           SPEED_OBSERVER_SAMPLE_FREQ_HZ);
        #else
        {
            /* 差分测速：计算角度差分（处理跨圈情况） */
            float delta_angle = mechanical_angle - g_speed_ctrl.last_angle;
            g_speed_ctrl.last_angle = mechanical_angle;
            if (delta_angle > PI) {
                delta_angle -= TWO_PI;
            } else if (delta_angle < -PI) {
                delta_angle += TWO_PI;
            }
            speed = LPF_Update(&g_speed_ctrl.observer_lpf,
                               delta_angle * SPEED_OBSERVER_SAMPLE_FREQ_HZ);
        }
        #endif
    }

    g_speed_ctrl.observer_speed_sum += speed;
    g_speed_ctrl.observer_sample_count++;
    return speed;
}

BOOL SpeedController_TryGetAveragedSpeed(float *averaged_speed)
{
    if ((averaged_speed == NULL) ||
        (g_speed_ctrl.observer_sample_count < SPEED_CTRL_DECIMATION))
    {
        return FALSE;
    }

    *averaged_speed = g_speed_ctrl.observer_speed_sum
                    / (float)g_speed_ctrl.observer_sample_count;
    g_speed_ctrl.observer_speed_sum = 0.0f;
    g_speed_ctrl.observer_sample_count = 0u;
    return TRUE;
}

float SpeedController_UpdateFromSpeed(float measured_speed)
{
    float residual;
    float shaped_residual;
    float residual_rho;
    float residual_deadband;

    g_speed_ctrl.fast_speed = measured_speed;
    g_speed_ctrl.trend_speed = LPF_Update(&g_speed_ctrl.trend_lpf, measured_speed);

    residual = g_speed_ctrl.trend_speed - measured_speed;
    residual_rho = g_speed_ctrl.params->residual_rho;
    residual_deadband = g_speed_ctrl.params->residual_deadband;

    if (residual > residual_deadband) {
        shaped_residual = residual - (1.0f - residual_rho) * residual_deadband;
    } else if (residual < -residual_deadband) {
        shaped_residual = residual + (1.0f - residual_rho) * residual_deadband;
    } else {
        shaped_residual = residual_rho * residual;
    }

    /* target - filtered_speed 等于趋势误差加非线性残差。 */
    g_speed_ctrl.filtered_speed = g_speed_ctrl.trend_speed - shaped_residual;

    /* 命令整形：每个速度环周期推进一次目标速度滤波 */
    g_speed_ctrl.target_speed = LPF_Update(&g_speed_ctrl.cmd_lpf, g_speed_ctrl.raw_target_speed);

    /* 比例项使用等效反馈，积分项只跟随慢速趋势。 */
    g_speed_ctrl.pi_output = PI_Update(&g_speed_ctrl.pi,
                                       g_speed_ctrl.target_speed,
                                       g_speed_ctrl.filtered_speed,
                                       g_speed_ctrl.trend_speed);

    /* 加速度前馈：对目标速度差分 */
    g_speed_ctrl.accel_ff = g_speed_ctrl.params->ff.acceleration_gain
                          * (g_speed_ctrl.target_speed - g_speed_ctrl.prev_target_speed)
                          * SPEED_CTRL_SAMPLE_FREQ_HZ;
    g_speed_ctrl.prev_target_speed = g_speed_ctrl.target_speed;

    /* 摩擦前馈：系数 * sign(target_speed) */
    if (g_speed_ctrl.target_speed > 0.0f) {
        g_speed_ctrl.friction_ff = g_speed_ctrl.params->ff.friction_compensation;
    } else if (g_speed_ctrl.target_speed < 0.0f) {
        g_speed_ctrl.friction_ff = -g_speed_ctrl.params->ff.friction_compensation;
    } else {
        g_speed_ctrl.friction_ff = 0.0f;
    }

    /* 总电流参考 = PI + 加速度前馈 + 负载前馈 + 摩擦前馈 */
    g_speed_ctrl.i_output = g_speed_ctrl.pi_output
                          + g_speed_ctrl.accel_ff
                          + g_speed_ctrl.load_ff
                          + g_speed_ctrl.friction_ff;

    /* 电压输出 = i_output * R + Ke * ω_ref */
    g_speed_ctrl.total_output = g_speed_ctrl.i_output * g_speed_ctrl.params->ff.R
                              + g_speed_ctrl.params->ff.Ke * g_speed_ctrl.target_speed;

    return g_speed_ctrl.total_output;
}

void SpeedController_Reset(void)
{
    PI_Reset(&g_speed_ctrl.pi);
    LPF_Reset(&g_speed_ctrl.observer_lpf);
    LPF_Reset(&g_speed_ctrl.trend_lpf);
    g_speed_ctrl.angle_initialized = 0;
    g_speed_ctrl.fast_speed = 0.0f;
    g_speed_ctrl.trend_speed = 0.0f;
    g_speed_ctrl.filtered_speed = 0.0f;
    g_speed_ctrl.last_angle = 0.0f;
    g_speed_ctrl.observer_speed_sum = 0.0f;
    g_speed_ctrl.observer_sample_count = 0u;
    g_speed_ctrl.pi_output = 0.0f;
    g_speed_ctrl.accel_ff = 0.0f;
    g_speed_ctrl.friction_ff = 0.0f;
    g_speed_ctrl.i_output = 0.0f;
    g_speed_ctrl.total_output = 0.0f;
    g_speed_ctrl.prev_target_speed = 0.0f;
    g_speed_ctrl.cmd_lpf.output = g_speed_ctrl.target_speed;
    g_speed_ctrl.cmd_lpf.init_done = 1;

#if SPEED_OBSERVER_METHOD == 1
    PLL_Reset(&g_speed_ctrl.pll, 0.0f);
#endif
}

void SpeedController_RefreshParams(SpeedController_Params_t *params)
{
    float raw_target_speed;
    float shaped_target_speed;

    raw_target_speed = g_speed_ctrl.raw_target_speed;
    shaped_target_speed = g_speed_ctrl.target_speed;
    SpeedController_Init(params);
    g_speed_ctrl.target_speed = shaped_target_speed;
    g_speed_ctrl.cmd_lpf.output = shaped_target_speed;
    g_speed_ctrl.cmd_lpf.init_done = 1;
    SpeedController_SetTarget(raw_target_speed);
}

void SpeedController_SetLoadFF(float load_ff)
{
    g_speed_ctrl.load_ff = load_ff;
}
