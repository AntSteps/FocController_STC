#include "config.h"

#include "svpwm.h"

/* 常量定义 */
#define SQRT3_2 0.86602540378f /* sqrt(3)/2 */
#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define ONE_OVER_2PI 0.15915494309f /* 1/(2*PI) */

//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 文件名称: svpwm.c
// 文件描述: SVPWM空间矢量脉宽调制驱动 (LUT 极速版)
// 文件版本: V2.0
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>

#include "config.h"

//<<AICUBE_USER_INCLUDE_BEGIN>>
#include "svpwm.h"
#include <math.h> // 仅在初始化建表时使用
//<<AICUBE_USER_INCLUDE_END>>

//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>

/* 查表法常量定义 */
#define SVPWM_LUT_SIZE  1024  // 表格大小
#define OFFSET_120_DEG  (SVPWM_LUT_SIZE / 3)      // Phase C 偏移 (+120°)
#define OFFSET_240_DEG  (SVPWM_LUT_SIZE * 2 / 3)  // Phase B 偏移 (-120° 等同于 +240°)
#define OFFSET_90_DEG   (SVPWM_LUT_SIZE / 4)      // Q轴 偏移 (+90°)

#define TWO_PI 6.28318530718f

/* 缓存区 */
static uint16_t s_pwm_period = 0;
static float s_half_period = 0.0f;

/* 马鞍波查找表，存放归一化(Q12格式)的波形数据 */
int16_t xdata s_svpwm_lut[SVPWM_LUT_SIZE];




// SVPWM初始化函数 建表
void SVPWM_Init(void)
{
    int i;
    float theta, v_a, v_b, v_c, v_max, v_min, v_offset, saddle_val;
    
    s_pwm_period = 399; 
    s_half_period = (float)(s_pwm_period + 1) * 0.5f;

    /* 动态生成 A相的基准马鞍波查表 */
    for (i = 0; i < SVPWM_LUT_SIZE; i++) 
    {
        theta = (float)i * TWO_PI / (float)SVPWM_LUT_SIZE;
        
        // 算出传统三相正弦波 (D轴对齐Alpha轴，即 A相为 cos(theta))
        v_a = cos(theta);
        v_b = cos(theta - TWO_PI / 3.0f);
        v_c = cos(theta + TWO_PI / 3.0f);

        // Min-Max 零序电压注入计算
        v_max = v_a > v_b ? (v_a > v_c ? v_a : v_c) : (v_b > v_c ? v_b : v_c);
        v_min = v_a < v_b ? (v_a < v_c ? v_a : v_c) : (v_b < v_c ? v_b : v_c);
        v_offset = -0.5f * (v_max + v_min);

        // 生成标准马鞍波 (峰值约为 0.866)
        saddle_val = v_a + v_offset;

        // 放大 4096 倍 (12位移位) 存入 int16_t 数组
        // 0.866 * 4096 ≈ 3547，完美落在 int16_t 范围内 (-32768 ~ 32767)
        s_svpwm_lut[i] = (int16_t)(saddle_val * 4096.0f);
    }
}


static void SVPWM_Update_Core(uint16_t base_index, float mag)
{
    int32_t mag_scaled;
    int32_t duty_a, duty_b, duty_c;
    uint16_t idx_a, idx_b, idx_c;

    // 获取三相的查表索引 (快速取模)
    idx_a = base_index % SVPWM_LUT_SIZE;
    idx_b = (base_index + OFFSET_240_DEG) % SVPWM_LUT_SIZE;
    idx_c = (base_index + OFFSET_120_DEG) % SVPWM_LUT_SIZE;

    // 将幅度与半周期混合，预先准备好乘数
    // 这一步把浮点彻底终结，接下来的计算全是定点整型
    mag_scaled = (int32_t)(mag * s_half_period);

    // 查表 -> 乘法 -> >>12 缩放 -> 加上 half_period 中点偏置
    // 公式推导: duty = half_period + ( (LUT_val / 4096) * (mag * half_period) )
    duty_a = (int32_t)s_half_period + ((s_svpwm_lut[idx_a] * mag_scaled) >> 12);
    duty_b = (int32_t)s_half_period + ((s_svpwm_lut[idx_b] * mag_scaled) >> 12);
    duty_c = (int32_t)s_half_period + ((s_svpwm_lut[idx_c] * mag_scaled) >> 12);

    // 极速限幅保护
    if (duty_a < 0) duty_a = 0; else if (duty_a > s_pwm_period) duty_a = s_pwm_period;
    if (duty_b < 0) duty_b = 0; else if (duty_b > s_pwm_period) duty_b = s_pwm_period;
    if (duty_c < 0) duty_c = 0; else if (duty_c > s_pwm_period) duty_c = s_pwm_period;
    // 更新寄存器
    PWM_UpdateDuty(PWMA_CH1, (uint16_t)duty_a);
    PWM_UpdateDuty(PWMA_CH2, (uint16_t)duty_b);
    PWM_UpdateDuty(PWMA_CH3, (uint16_t)duty_c);
}


void SVPWM_Update(float angle_el, float mag)
{

    int32_t angle_int = (int32_t)(angle_el * ((float)SVPWM_LUT_SIZE / TWO_PI));
    uint16_t base_index = (uint16_t)((angle_int % SVPWM_LUT_SIZE + SVPWM_LUT_SIZE) % SVPWM_LUT_SIZE);

    SVPWM_Update_Core(base_index, mag);
}


void SVPWM_UpdateQAxis(float angle_el, float mag)
{
    int32_t angle_int = (int32_t)(angle_el * ((float)SVPWM_LUT_SIZE / TWO_PI));
    uint16_t base_index = (uint16_t)((angle_int % SVPWM_LUT_SIZE + SVPWM_LUT_SIZE) % SVPWM_LUT_SIZE);
    
    base_index += OFFSET_90_DEG;

    SVPWM_Update_Core(base_index, mag);
}