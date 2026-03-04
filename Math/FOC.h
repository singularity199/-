/**
  ******************************************************************************
  * @file    foc.h
  * @brief   FOC 核心控制逻辑头文件
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

#define _PI                 3.14159265f
#define _2PI                6.28318530f
#define _SQRT3              1.73205081f
#define _ONE_DIV_SQRT3      0.57735027f

// 调试阶段开环/锁轴电压限幅 (V)
#define OPENLOOP_VOLTAGE_LIMIT 2.0f

typedef enum {
    FOC_STATE_IDLE,
    FOC_STATE_ALIGN,
    FOC_STATE_OPEN_LOOP,
    FOC_STATE_RUN,
    FOC_STATE_FAULT
} FOC_State_Enum;

typedef struct {
    float Kp;
    float Ki;
    float integral_err;
    float out_limit;
    float int_limit;
} PID_Controller_t;

typedef struct {
    // --- 状态机 ---
    FOC_State_Enum State;
    uint32_t state_counter;

    // --- 启动流程参数 ---
    uint8_t  startup_enable;         // 1: 自动按 IDLE->ALIGN->OPEN_LOOP 走
    uint32_t idle_cycles;            // IDLE 保持周期数
    uint32_t align_cycles;           // ALIGN 保持周期数
    uint32_t openloop_cycles;        // OPEN_LOOP 保持周期数 (0=不自动切换)

    // --- 开环角度发生器 ---
    float Target_Speed;              // OPEN_LOOP 目标电角速度 (rad/s)
    float Ramp_Angle;                // 开环角度 (rad)
    float Ramp_Speed;                // 当前电角速度 (rad/s)
    float OpenLoop_Accel;            // 开环加速度 (rad/s^2)

    // --- 开环/定位直接电压 ---
    float OpenLoop_Vd;
    float OpenLoop_Vq;
    float Align_Vd;
    float Align_Vq;

    // --- 反馈 ---
    float I_u;
    float I_v;
    float I_w;
    float V_bus;

    // --- 变换变量 ---
    float Theta;
    float Sin_Theta;
    float Cos_Theta;
    float I_alpha;
    float I_beta;
    float I_d;
    float I_q;

    // --- 控制目标 ---
    float Id_target;
    float Iq_target;

    // --- 输出 ---
    float V_d;
    float V_q;
    float V_alpha;
    float V_beta;

} FOC_Handle_t;

extern FOC_Handle_t FOC;
extern PID_Controller_t PID_Id;
extern PID_Controller_t PID_Iq;

void FOC_Init(void);
void FOC_Loop_ISR(void);
void FOC_Set_Target(float iq, float id);

#endif /* __FOC_H */
