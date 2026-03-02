/**
  ******************************************************************************
  * @file    foc.h
  * @brief   FOC 核心控制逻辑头文件
  * @note    包含状态机定义、PID 参数、核心运算结构体
  ******************************************************************************
  */

#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include <math.h>

/* ==============================================================================
 * 宏定义与常量
 * ============================================================================== */
#define FOC_CTRL_FREQ       20000.0f             // 控制频率 20kHz
#define FOC_DT              (1.0f/FOC_CTRL_FREQ) // 采样周期 50us

// 数学常量
#define _PI                 3.14159265f
#define _2PI                6.28318530f
#define _SQRT3              1.73205081f
#define _ONE_DIV_SQRT3      0.57735027f

/* ==============================================================================
 * 数据结构定义
 * ============================================================================== */

// FOC 运行状态枚举
typedef enum {
    FOC_STATE_IDLE,         // 待机 (PWM 关闭)
    FOC_STATE_ALIGN,        // 预定位 (固定角度通电)
    FOC_STATE_OPEN_LOOP,    // 开环强拖 (Ramp-up)
    FOC_STATE_RUN,          // 闭环运行 (Closed Loop)
    FOC_STATE_FAULT         // 故障停机
} FOC_State_Enum;

// PID 控制器结构体
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float integral_err;     // 积分累积误差
    float out_limit;        // 输出限幅 (电压 V)
    float int_limit;        // 积分限幅
} PID_Controller_t;

// FOC 全局状态结构体 (God Object)
typedef struct {
    // --- 1. 状态机控制 ---
    FOC_State_Enum State;
    float Target_Speed;     // 目标电角速度 (RPM)
    float Ramp_Angle;       // 开环生成的虚拟角度 (rad)
    float Ramp_Speed;       // 当前开环电角速度 (RPM)
    float Ramp_Accel;       // 开环电角加速度 (RPM/s)
    float Ramp_Speed_Min;   // 开环初始电角速度 (RPM)

    // --- 启动流程控制 ---
    uint32_t align_count;   // ALIGN 已执行周期计数
    uint32_t align_cycles;  // ALIGN 目标周期数
    uint8_t OpenLoop_UseVq; // 1: 开环固定Vq测试; 0: 开环电流环
    float OpenLoop_Vq;      // 固定Vq测试电压 (V)
    float Align_Vd;         // ALIGN阶段固定Vd (V)
    float Align_Vq;         // ALIGN阶段固定Vq (V)

    // --- 2. 传感器反馈 (Input) ---
    float I_u;              // U相电流 (A)
    float I_v;              // V相电流 (A)
    float I_w;              // W相电流 (A)
    float V_bus;            // 母线电压 (V) - 暂时固定，以后用 ADC 采

    // --- 3. 核心变换变量 (Math) ---
    float Theta;            // 电角度 (rad) [0, 2PI]
    float Sin_Theta;        // sin(Theta)
    float Cos_Theta;        // cos(Theta)
    
    // Clarke 变换结果 (静止坐标系)
    float I_alpha;
    float I_beta;
    
    // Park 变换结果 (旋转坐标系 - 反馈值)
    float I_d;
    float I_q;

    // --- 4. 控制目标 (Setpoints) ---
    float Id_target;        // D轴目标电流 (通常为0，启动时可设为强拖电流)
    float Iq_target;        // Q轴目标电流 (力矩/油门)

    // --- 5. 控制输出 (Output) ---
    // PID 计算出的电压指令 (旋转坐标系)
    float V_d;
    float V_q;
    
    // 反 Park 变换结果 (静止坐标系)
    float V_alpha;
    float V_beta;

} FOC_Handle_t;

/* ==============================================================================
 * 外部变量声明
 * ============================================================================== */
extern FOC_Handle_t FOC;    // 核心对象
extern PID_Controller_t PID_Id; // D轴 PID
extern PID_Controller_t PID_Iq; // Q轴 PID

/* ==============================================================================
 * 函数声明
 * ============================================================================== */
void FOC_Init(void);
void FOC_Loop_ISR(void); // 核心中断处理函数 (放在 ADC 中断里调)

// 辅助函数：设置目标电流
void FOC_Set_Target(float iq, float id);

#endif /* __FOC_H */
