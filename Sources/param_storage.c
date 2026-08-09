//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 文件名称: param_storage.c
// 文件描述: 参数持久化管理模块实现
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-17) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#include "config.h"
#include "param_storage.h"
#include "crc8.h"


// EEPROM扇区大小 (STC8H系列通常为512字节)
#define EEPROM_SECTOR_SIZE      512


////////////////////////////////////////
// 静态变量定义 (单例模式)
////////////////////////////////////////

// 参数数据工作副本
static ParamStorage_Data_t s_param_data;

// 标志位
static BOOL s_initialized = FALSE;
static BOOL s_loaded_from_eeprom = FALSE;


////////////////////////////////////////
// 内部函数
////////////////////////////////////////

/* 从EEPROM读取数据块 */
static void EEPROM_ReadBuffer(uint32 addr, uint8 *buf, uint16 len)
{
    uint16 i;
    for (i = 0; i < len; i++) {
        buf[i] = IAP_ReadByte(addr + i);
    }
}

/* 写入数据块到EEPROM (需要先擦除) */
static void EEPROM_WriteBuffer(uint32 addr, const uint8 *buf, uint16 len)
{
    uint16 i;
    for (i = 0; i < len; i++) {
        IAP_ProgramByte(addr + i, buf[i]);
    }
}

// 设置默认值
static void SetDefaultValues(void)
{
    memset(&s_param_data, 0, sizeof(ParamStorage_Data_t));

    s_param_data.magic = PARAM_STORAGE_MAGIC;
    s_param_data.version = PARAM_STORAGE_VERSION;

    // 电机默认参数
    s_param_data.motor_pole_pairs = 7;          // 默认7对极
    s_param_data.motor_phase_resistance = 6.0f; // 默认0.1欧姆
    s_param_data.motor_phase_inductance = 0.0001f; // 默认100uH


    // 速度环默认参数
    s_param_data.speed_ctrl_params.kp = 0.033f;
    s_param_data.speed_ctrl_params.ki = 0.004f;
    s_param_data.speed_ctrl_params.beta = 0.5f;  /* 二自由度系数 */
    s_param_data.speed_ctrl_params.out_max = 1.5f;
    s_param_data.speed_ctrl_params.out_min = -1.5f;
    s_param_data.speed_ctrl_params.lpf_time_constant_ms = 160.0f; /* 约1Hz趋势低通 */
    s_param_data.speed_ctrl_params.cmd_lpf_time_constant_ms = 10.0f;  /* 命令整形滤波*/
    s_param_data.speed_ctrl_params.pll_bandwidth_hz = 30.0f;
    s_param_data.speed_ctrl_params.pll_zeta = 0.707f;
    s_param_data.speed_ctrl_params.residual_rho = 0.25f;
    s_param_data.speed_ctrl_params.residual_deadband = 2.3f;   /* rad/s */
    s_param_data.speed_ctrl_params.ff.acceleration_gain = 0.0f;
    s_param_data.speed_ctrl_params.ff.friction_compensation = 0.023f;
    s_param_data.speed_ctrl_params.ff.R = 6.2f;        /* 相电阻前馈 */
    s_param_data.speed_ctrl_params.ff.Ke = 0.0359f;       /* 反电动势系数 */

    // 默认限制值

    s_param_data.velocity_limit = 1000.0f;      // 1000 rad/s

    // 编码器默认参数
    s_param_data.encoder_offset = 0;

    // 电机校准默认参数
    s_param_data.motor_calib.pole_pairs = 7;
    s_param_data.motor_calib.dir = -1;
    s_param_data.motor_calib.electrical_offset_rad = 0.0f;
    s_param_data.motor_calib.harmonic_amp_1 = 0.0f;
    s_param_data.motor_calib.harmonic_phase_1 = 0.0f;
    s_param_data.motor_calib.harmonic_amp_2 = 0.0f;
    s_param_data.motor_calib.harmonic_phase_2 = 0.0f;

    // CRC8会在保存时自动计算
    s_param_data.crc8 = 0;
}


////////////////////////////////////////
// 接口函数实现
////////////////////////////////////////

// 初始化参数存储模块
void ParamStorage_Init(void)
{
    if (s_initialized) {
        return;
    }

    // 尝试从EEPROM加载
    if (!ParamStorage_Load()) {
        s_loaded_from_eeprom = FALSE;
        // 加载失败，使用默认值并保存
        SetDefaultValues();
        ParamStorage_Save();
    } else {
        s_loaded_from_eeprom = TRUE;
    }

    s_initialized = TRUE;
}

// 从EEPROM加载参数
BOOL ParamStorage_Load(void)
{
    ParamStorage_Data_t temp;
    uint8 calc_crc;

    // 从EEPROM读取数据
    EEPROM_ReadBuffer(PARAM_STORAGE_EEPROM_ADDR, (uint8 *)&temp, sizeof(ParamStorage_Data_t));

    // 校验魔数
    if (temp.magic != PARAM_STORAGE_MAGIC) {
        return FALSE;
    }

    // 校验版本号，版本不匹配也视为无效
    if (temp.version != PARAM_STORAGE_VERSION) {
        return FALSE;
    }

    // CRC8校验 (校验除crc8字段外的所有数据)
    calc_crc = Mem_CRC8_Check((uint8 *)&temp, sizeof(ParamStorage_Data_t) - sizeof(temp.crc8), 0);
    if (calc_crc != temp.crc8) {
        return FALSE;
    }

    // 数据有效，复制到工作副本
    s_param_data = temp;
    return TRUE;
}

// 保存参数到EEPROM
BOOL ParamStorage_Save(void)
{
    // 确保魔数和版本正确
    s_param_data.magic = PARAM_STORAGE_MAGIC;
    s_param_data.version = PARAM_STORAGE_VERSION;

    // 计算CRC8 (校验除crc8字段外的所有数据)
    s_param_data.crc8 = Mem_CRC8_Check((uint8 *)&s_param_data,
                                       sizeof(ParamStorage_Data_t) - sizeof(s_param_data.crc8),
                                       0);

    // 擦除EEPROM扇区
    IAP_EraseSector(PARAM_STORAGE_EEPROM_ADDR);

    // 写入数据
    EEPROM_WriteBuffer(PARAM_STORAGE_EEPROM_ADDR, (const uint8 *)&s_param_data, sizeof(ParamStorage_Data_t));

    // 可选：验证写入 (这里简化处理，假设写入成功)
    return TRUE;
}

// 重置为默认值
void ParamStorage_ResetToDefault(void)
{
    SetDefaultValues();
}

// 获取参数结构体指针
ParamStorage_Data_t* ParamStorage_Get(void)
{
    return &s_param_data;
}

BOOL ParamStorage_WasLoadedFromEEPROM(void)
{
    return s_loaded_from_eeprom;
}
