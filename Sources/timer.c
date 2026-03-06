//<<AICUBE_USER_HEADER_REMARK_BEGIN>>
////////////////////////////////////////
// 在此添加用户文件头说明信息  
// 文件名称: timer.c
// 文件描述: 
// 文件版本: V1.0
// 修改记录:
//   1. (2026-03-06) 创建文件
////////////////////////////////////////
//<<AICUBE_USER_HEADER_REMARK_END>>


#include "config.h"


//<<AICUBE_USER_INCLUDE_BEGIN>>
// 在此添加用户头文件包含  
//<<AICUBE_USER_INCLUDE_END>>


//<<AICUBE_USER_GLOBAL_DEFINE_BEGIN>>
// 在此添加用户全局变量定义、用户宏定义以及函数声明  
//<<AICUBE_USER_GLOBAL_DEFINE_END>>



////////////////////////////////////////
// 定时器3初始化函数
// 入口参数: 无
// 函数返回: 无
////////////////////////////////////////
void TIMER3_Init(void)
{
#define T3_PSCR                 (0)
#ifdef BAUDRATE
#undef BAUDRATE
#endif
#define BAUDRATE                (115200)
#define T3_RELOAD               (65536 - (SYSCLK / BAUDRATE + 2) / 4)

    TIMER3_TimerMode();                 //设置定时器3为定时模式
    TIMER3_1TMode();                    //设置定时器3为1T模式
    TIMER3_SetPrescale(T3_PSCR);        //设置定时器3的8位预分频
    TIMER3_SetReload16(T3_RELOAD);      //设置定时器3的16位重载值
    TIMER3_Run();                       //定时器3开始运行

    //<<AICUBE_USER_TIMER3_INITIAL_BEGIN>>
    // 在此添加用户初始化代码  
    //<<AICUBE_USER_TIMER3_INITIAL_END>>
}



//<<AICUBE_USER_FUNCTION_IMPLEMENT_BEGIN>>
// 在此添加用户函数实现代码  
//<<AICUBE_USER_FUNCTION_IMPLEMENT_END>>


