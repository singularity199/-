/**
  ******************************************************************************
  * @file    foc.c
  * @brief   FOC 核心算法实现
  * @note    包含 Clarke, Park, PID, InvPark, Angle Generator
  ******************************************************************************
  */

#include "foc.h"
#include "adc.h"       // 获取电流
#include "foc_svpwm.h" // 输出 PWM
#include "tim.h"       // 控制 PWM 开关

/* ==============================================================================
 * 全局变量定义
 * ============================================================================== */
FOC_Handle_t FOC;

// 定义 PID 参数 (需根据电机调整)
// 经验值：小电机 Kp=0.1~1.0, Ki=10~100; 大电机参数更大
PID_Controller_t PID_Id = {
    .Kp = 0.5f,
    .Ki = 50.0f,
    .out_limit = 12.0f, // 假设 3S 电池 (12V)
    .int_limit = 6.0f
};

PID_Controller_t PID_Iq = {
    .Kp = 0.5f,
    .Ki = 50.0f,
    .out_limit = 12.0f,
    .int_limit = 6.0f
};

/* ==============================================================================
 * 内部辅助函数
 * ============================================================================== */

/**
 * @brief  PID 计算函数 (增量式或位置式)
 * @param  pid: PID 结构体指针
 * @param  target: 目标值
 * @param  measurement: 实际测量值
 * @return 输出电压 (V)
 */
static float PID_Calc(PID_Controller_t *pid, float target, float measurement) {
    float error = target - measurement;

    // 1. 积分项计算
    pid->integral_err += error * FOC_DT * pid->Ki;

    // 2. 积分抗饱和 (Anti-windup)
    if (pid->integral_err > pid->int_limit)  pid->integral_err = pid->int_limit;
    if (pid->integral_err < -pid->int_limit) pid->integral_err = -pid->int_limit;

    // 3. 计算总输出
    float output = (error * pid->Kp) + pid->integral_err;

    // 4. 输出限幅
    if (output > pid->out_limit)  output = pid->out_limit;
    if (output < -pid->out_limit) output = -pid->out_limit;

    return output;
}

/**
 * @brief  开环角度发生器 (Ramp Generator)
 * @note   用于开环强拖启动
 */
static void Angle_Generator_Update(void) {
    // 按加速度做速度爬升（防止一步到高速）
    if (FOC.Ramp_Speed < FOC.Target_Speed) {
        FOC.Ramp_Speed += FOC.Ramp_Accel * FOC_DT;
        if (FOC.Ramp_Speed > FOC.Target_Speed) {
            FOC.Ramp_Speed = FOC.Target_Speed;
        }
    }

    // 简单的速度积分: Theta = Theta + Omega * dt
    FOC.Ramp_Angle += FOC.Ramp_Speed * FOC_DT;

    // 归一化到 [0, 2PI]
    if (FOC.Ramp_Angle > _2PI) FOC.Ramp_Angle -= _2PI;
    if (FOC.Ramp_Angle < 0.0f) FOC.Ramp_Angle += _2PI;
}

/* ==============================================================================
 * 核心 API
 * ============================================================================== */

/**
 * @brief  FOC 初始化
 */
void FOC_Init(void) {
    float max_voltage;

    // 初始化变量
    FOC.State = FOC_STATE_IDLE;
    FOC.V_bus = 12.6f; // 默认 3S 满电，建议后续用 ADC 采集实际电压

    // 开环参数
    FOC.Ramp_Angle = 0.0f;
    FOC.Ramp_Speed = 0.0f;
    FOC.Ramp_Speed_Min = 3.0f;     // 从很低电角速度起步 (rad/s)
    FOC.Ramp_Accel = 40.0f;        // 电角加速度 (rad/s^2)

    // ALIGN 参数（约 200ms）
    FOC.align_count = 0;
    FOC.align_cycles = (uint32_t)(FOC_CTRL_FREQ * 0.2f);
    FOC.Align_Vd = 0.8f;
    FOC.Align_Vq = 0.0f;

    // 开环验证模式：固定小Vq（先断开电流环验证能否带动）
    FOC.OpenLoop_UseVq = 1;
    FOC.OpenLoop_Vq = 1.0f;

    // 目标默认值
    FOC.Target_Speed = FOC.Ramp_Speed_Min;
    FOC.Id_target = 0.0f;
    FOC.Iq_target = 0.3f;

    // 限制 PID 输出不能超过母线电压的 1/sqrt(3) (SVPWM 线性区极限)
    max_voltage = FOC.V_bus * _ONE_DIV_SQRT3;
    PID_Id.out_limit = max_voltage;
    PID_Iq.out_limit = max_voltage;
    PID_Id.integral_err = 0.0f;
    PID_Iq.integral_err = 0.0f;
}

/**
 * @brief  设置控制目标
 */
void FOC_Set_Target(float iq, float id) {
    FOC.Iq_target = iq;
    FOC.Id_target = id;
}

/**
 * @brief  FOC 核心中断服务函数
 * @note   必须在 ADC 注入中断 (JEOC) 中调用
 * @time   耗时应控制在 10us 以内
 */
void FOC_Loop_ISR(void) {

    /* --------------------------------------------------------------------------
     * 1. 获取电流 (Measurement)
     * -------------------------------------------------------------------------- */
    // 实际工程中通常在 ADC Handler 里直接赋值给 FOC 结构体

    /* --------------------------------------------------------------------------
     * 2. Clarke 变换 (3相 -> 2相静止)
     * -------------------------------------------------------------------------- */
    FOC.I_alpha = FOC.I_u;
    FOC.I_beta  = (FOC.I_u + 2.0f * FOC.I_v) * _ONE_DIV_SQRT3;

    /* --------------------------------------------------------------------------
     * 3. 角度与启动状态机
     * -------------------------------------------------------------------------- */
    if (FOC.State == FOC_STATE_ALIGN) {
        // 固定电角度，用小电压建立定子磁场锁定转子
        FOC.Theta = 0.0f;
        FOC.V_d = FOC.Align_Vd;
        FOC.V_q = FOC.Align_Vq;

        FOC.align_count++;
        if (FOC.align_count >= FOC.align_cycles) {
            FOC.align_count = 0;
            FOC.Ramp_Angle = 0.0f;
            FOC.Ramp_Speed = FOC.Ramp_Speed_Min;
            PID_Id.integral_err = 0.0f;
            PID_Iq.integral_err = 0.0f;
            FOC.State = FOC_STATE_OPEN_LOOP;
        }
    }
    else if (FOC.State == FOC_STATE_OPEN_LOOP) {
        // 开环模式：速度从很低值开始爬升
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle;

        if (FOC.OpenLoop_UseVq) {
            // 先断开电流环验证：固定小 Vq
            FOC.V_d = 0.0f;
            FOC.V_q = FOC.OpenLoop_Vq;
        } else {
            // 回到电流环控制
            FOC.Sin_Theta = sinf(FOC.Theta);
            FOC.Cos_Theta = cosf(FOC.Theta);
            FOC.I_d =  FOC.I_alpha * FOC.Cos_Theta + FOC.I_beta * FOC.Sin_Theta;
            FOC.I_q = -FOC.I_alpha * FOC.Sin_Theta + FOC.I_beta * FOC.Cos_Theta;
            FOC.V_d = PID_Calc(&PID_Id, FOC.Id_target, FOC.I_d);
            FOC.V_q = PID_Calc(&PID_Iq, FOC.Iq_target, FOC.I_q);
        }
    }
    else if (FOC.State == FOC_STATE_RUN) {
        // 闭环模式：临时仍用开环角度（后续接入观测器）
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle;

        FOC.Sin_Theta = sinf(FOC.Theta);
        FOC.Cos_Theta = cosf(FOC.Theta);
        FOC.I_d =  FOC.I_alpha * FOC.Cos_Theta + FOC.I_beta * FOC.Sin_Theta;
        FOC.I_q = -FOC.I_alpha * FOC.Sin_Theta + FOC.I_beta * FOC.Cos_Theta;
        FOC.V_d = PID_Calc(&PID_Id, FOC.Id_target, FOC.I_d);
        FOC.V_q = PID_Calc(&PID_Iq, FOC.Iq_target, FOC.I_q);
    }
    else {
        // IDLE / FAULT
        PWM_Timer_Stop();
        return;
    }

    // 仅在上面没算 sin/cos 的分支补算
    if (FOC.State == FOC_STATE_ALIGN || (FOC.State == FOC_STATE_OPEN_LOOP && FOC.OpenLoop_UseVq)) {
        FOC.Sin_Theta = sinf(FOC.Theta);
        FOC.Cos_Theta = cosf(FOC.Theta);
    }

    /* --------------------------------------------------------------------------
     * 7. 反 Park 变换 (2相旋转 -> 2相静止)
     * -------------------------------------------------------------------------- */
    FOC.V_alpha = FOC.V_d * FOC.Cos_Theta - FOC.V_q * FOC.Sin_Theta;
    FOC.V_beta  = FOC.V_d * FOC.Sin_Theta + FOC.V_q * FOC.Cos_Theta;

    /* --------------------------------------------------------------------------
     * 8. SVPWM 生成 (Output)
     * -------------------------------------------------------------------------- */
    FOC_SVPWM_Update(FOC.V_alpha, FOC.V_beta, FOC.V_bus);
}
