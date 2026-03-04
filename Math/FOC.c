/**
  ******************************************************************************
  * @file    foc.c
  * @brief   FOC 核心算法实现
  ******************************************************************************
  */

#include "foc.h"
#include "adc.h"
#include "foc_svpwm.h"
#include "tim.h"

FOC_Handle_t FOC;

PID_Controller_t PID_Id = {
    .Kp = 0.5f,
    .Ki = 50.0f,
    .out_limit = 12.0f,
    .int_limit = 6.0f
};

PID_Controller_t PID_Iq = {
    .Kp = 0.5f,
    .Ki = 50.0f,
    .out_limit = 12.0f,
    .int_limit = 6.0f
};

static float PID_Calc(PID_Controller_t *pid, float target, float measurement)
{
    float error = target - measurement;

    pid->integral_err += error * FOC_DT * pid->Ki;

    if (pid->integral_err > pid->int_limit)  pid->integral_err = pid->int_limit;
    if (pid->integral_err < -pid->int_limit) pid->integral_err = -pid->int_limit;

    float output = (error * pid->Kp) + pid->integral_err;

    if (output > pid->out_limit)  output = pid->out_limit;
    if (output < -pid->out_limit) output = -pid->out_limit;

    return output;
}

static float Clamp_OpenLoopVoltage(float v)
{
    if (v > OPENLOOP_VOLTAGE_LIMIT)  return OPENLOOP_VOLTAGE_LIMIT;
    if (v < -OPENLOOP_VOLTAGE_LIMIT) return -OPENLOOP_VOLTAGE_LIMIT;
    return v;
}

static void Angle_Generator_Update(void)
{
    if (FOC.Ramp_Speed < FOC.Target_Speed) {
        FOC.Ramp_Speed += FOC.OpenLoop_Accel * FOC_DT;
        if (FOC.Ramp_Speed > FOC.Target_Speed) {
            FOC.Ramp_Speed = FOC.Target_Speed;
        }
    }

    FOC.Ramp_Angle += FOC.Ramp_Speed * FOC_DT;

    if (FOC.Ramp_Angle > _2PI) FOC.Ramp_Angle -= _2PI;
    if (FOC.Ramp_Angle < 0.0f) FOC.Ramp_Angle += _2PI;
}

void FOC_Init(void)
{
    float max_voltage;

    FOC.State = FOC_STATE_IDLE;
    FOC.state_counter = 0;

    FOC.startup_enable = 1;
    FOC.idle_cycles = (uint32_t)(FOC_CTRL_FREQ * 0.2f);      // 200ms
    FOC.align_cycles = (uint32_t)(FOC_CTRL_FREQ * 1.0f);     // 1s
    FOC.openloop_cycles = 0;                                  // 不自动切RUN

    FOC.V_bus = 25.0f; // 调试母线电压

    FOC.Ramp_Angle = 0.0f;
    FOC.Ramp_Speed = 0.0f;
    FOC.Target_Speed = 25.0f;
    FOC.OpenLoop_Accel = 80.0f;

    FOC.Align_Vd = 1.0f;
    FOC.Align_Vq = 0.0f;
    FOC.OpenLoop_Vd = 0.0f;
    FOC.OpenLoop_Vq = 0.8f;

    FOC.Id_target = 0.0f;
    FOC.Iq_target = 0.0f;

    max_voltage = FOC.V_bus * _ONE_DIV_SQRT3;
    PID_Id.out_limit = max_voltage;
    PID_Iq.out_limit = max_voltage;
}

void FOC_Set_Target(float iq, float id)
{
    FOC.Iq_target = iq;
    FOC.Id_target = id;
}

void FOC_Loop_ISR(void)
{
    FOC.I_alpha = FOC.I_u;
    FOC.I_beta  = (FOC.I_u + 2.0f * FOC.I_v) * _ONE_DIV_SQRT3;

    // 自动启动状态机
    if (FOC.startup_enable) {
        if (FOC.State == FOC_STATE_IDLE && FOC.state_counter >= FOC.idle_cycles) {
            FOC.State = FOC_STATE_ALIGN;
            FOC.state_counter = 0;
        } else if (FOC.State == FOC_STATE_ALIGN && FOC.state_counter >= FOC.align_cycles) {
            FOC.State = FOC_STATE_OPEN_LOOP;
            FOC.state_counter = 0;
            FOC.Ramp_Angle = 0.0f;
            FOC.Ramp_Speed = 0.0f;
        } else if (FOC.State == FOC_STATE_OPEN_LOOP && FOC.openloop_cycles > 0 && FOC.state_counter >= FOC.openloop_cycles) {
            FOC.State = FOC_STATE_RUN;
            FOC.state_counter = 0;
        }
    }

    if (FOC.State == FOC_STATE_IDLE) {
        FOC.Theta = 0.0f;
        FOC.V_d = 0.0f;
        FOC.V_q = 0.0f;
    }
    else if (FOC.State == FOC_STATE_ALIGN) {
        FOC.Theta = 0.0f;
        FOC.V_d = Clamp_OpenLoopVoltage(FOC.Align_Vd);
        FOC.V_q = Clamp_OpenLoopVoltage(FOC.Align_Vq);
    }
    else if (FOC.State == FOC_STATE_OPEN_LOOP) {
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle;
        FOC.V_d = Clamp_OpenLoopVoltage(FOC.OpenLoop_Vd);
        FOC.V_q = Clamp_OpenLoopVoltage(FOC.OpenLoop_Vq);
    }
    else if (FOC.State == FOC_STATE_RUN) {
        // 当前仍无观测器，这里先延用开环角度
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle;
    }
    else {
        PWM_Timer_Stop();
        return;
    }

    FOC.Sin_Theta = sinf(FOC.Theta);
    FOC.Cos_Theta = cosf(FOC.Theta);

    FOC.I_d =  FOC.I_alpha * FOC.Cos_Theta + FOC.I_beta * FOC.Sin_Theta;
    FOC.I_q = -FOC.I_alpha * FOC.Sin_Theta + FOC.I_beta * FOC.Cos_Theta;

    if (FOC.State == FOC_STATE_RUN) {
        FOC.V_d = PID_Calc(&PID_Id, FOC.Id_target, FOC.I_d);
        FOC.V_q = PID_Calc(&PID_Iq, FOC.Iq_target, FOC.I_q);
    }

    FOC.V_alpha = FOC.V_d * FOC.Cos_Theta - FOC.V_q * FOC.Sin_Theta;
    FOC.V_beta  = FOC.V_d * FOC.Sin_Theta + FOC.V_q * FOC.Cos_Theta;

    FOC_SVPWM_Update(FOC.V_alpha, FOC.V_beta, FOC.V_bus);

    FOC.state_counter++;
}
