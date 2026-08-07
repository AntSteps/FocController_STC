#ifndef __PARAM_STORAGE_H__
#define __PARAM_STORAGE_H__


#include "def.h"
#include "control.h"
#include "calibration.h"


// EEPROM参数存储起始地址 (STC8H系列EEPROM起始地址)
#define PARAM_STORAGE_EEPROM_ADDR   0x0000

// 参数魔数，用于校验数据有效性
#define PARAM_STORAGE_MAGIC         0x5A5A

// 参数版本号，数据结构改变时需要更新
#define PARAM_STORAGE_VERSION       0x0004


////////////////////////////////////////
// 参数结构体定义
// 所有需要持久化的参数都放在这里
////////////////////////////////////////
#pragma pack(1)
typedef struct {
    // 魔数和版本 (用于校验)
    uint16 magic;
    uint16 version;

    // 电机基础参数
    int16 motor_pole_pairs;
    float motor_phase_resistance;
    float motor_phase_inductance;



    SpeedController_Params_t speed_ctrl_params;   /* 速度环参数 */

    // 运行限制参数
    float velocity_limit;

    // 编码器参数
    int16 encoder_offset;

    // 电机校准参数（包含offset和谐波补偿参数）
    Motor_Calibration motor_calib;

    // CRC8校验 (放在最后，校验前面所有数据)
    uint8  crc8;

} ParamStorage_Data_t;
#pragma pack()



////////////////////////////////////////
// 接口函数声明
////////////////////////////////////////

// 初始化参数存储模块，从EEPROM加载参数
// 如果EEPROM中没有有效数据，则使用默认值
void ParamStorage_Init(void);

// 从EEPROM加载参数到内存
// 返回TRUE表示成功，FALSE表示EEPROM中无有效数据
BOOL ParamStorage_Load(void);

// 将当前参数保存到EEPROM
// 返回TRUE表示成功
BOOL ParamStorage_Save(void);

// 重置为默认值
void ParamStorage_ResetToDefault(void);

// 获取参数结构体指针，用于直接访问和修改参数
// 修改后需要手动调用 ParamStorage_Save() 保存
ParamStorage_Data_t* ParamStorage_Get(void);

// 返回本次上电是否成功从EEPROM加载到有效参数
BOOL ParamStorage_WasLoadedFromEEPROM(void);


#endif // __PARAM_STORAGE_H__
