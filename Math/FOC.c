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
    
    // 1. 积分项计算 (Trapezoidal integration or Simple Rectangular)
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
    // 初始化变量
    FOC.State = FOC_STATE_IDLE;
    FOC.V_bus = 12.6f; // 默认 3S 满电，建议后续用 ADC 采集实际电压
    FOC.Ramp_Angle = 0.0f;
    FOC.Ramp_Speed = 0.0f;
    FOC.OpenLoop_Vd = 0.0f;
    FOC.OpenLoop_Vq = 0.8f;
    
    // 限制 PID 输出不能超过母线电压的 1/sqrt(3) (SVPWM 线性区极限)
    float max_voltage = FOC.V_bus * _ONE_DIV_SQRT3;
    PID_Id.out_limit = max_voltage;
    PID_Iq.out_limit = max_voltage;
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
    // 从 foc_adc.c 的全局变量中获取校准后的电流
    // 注意：假设你的 ADC 已经在运行并写入了这些变量
    // 在三电阻低侧采样中，I_u + I_v + I_w = 0
    // 我们直接读取 ADC 转换结果并乘以比例系数 (在 ADC IRQ Handler 里处理更好，这里假设直接拿结果)
    
    // 这里为了演示，假设我们在 foc_adc.c 里已经算好了 FOC.I_u 等
    // 实际工程中，通常在 ADC Handler 里直接赋值给 FOC 结构体
    
    /* --------------------------------------------------------------------------
     * 2. Clarke 变换 (3相 -> 2相静止)
     * -------------------------------------------------------------------------- */
    // I_alpha = I_u
    // I_beta  = (I_u + 2*I_v) / sqrt(3)
    FOC.I_alpha = FOC.I_u;
    FOC.I_beta  = (FOC.I_u + 2.0f * FOC.I_v) * _ONE_DIV_SQRT3;

    /* --------------------------------------------------------------------------
     * 3. 角度与速度处理 (Observer / Estimator)
     * -------------------------------------------------------------------------- */
    if (FOC.State == FOC_STATE_OPEN_LOOP) {
        // 开环模式：强制让角度旋转
        FOC.Ramp_Speed = FOC.Target_Speed; // 比如 100 rad/s
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle;

        // 关键：开环不走电流PID，直接使用主程序给定的电压指令
        FOC.V_d = FOC.OpenLoop_Vd;
        FOC.V_q = FOC.OpenLoop_Vq;
    }
    else if (FOC.State == FOC_STATE_RUN) {
        // 闭环模式：应该使用观测器 (SMO/Flux)
        // [TODO] 这里暂时用开环角度代替，实际需要加入观测器代码
        Angle_Generator_Update();
        FOC.Theta = FOC.Ramp_Angle; 
    }
    else if (FOC.State == FOC_STATE_ALIGN) {
        // 定位模式：角度固定为 0
        FOC.Theta = 0.0f;
    }
    else {
        // IDLE 模式，不计算
        PWM_Timer_Stop(); // 确保 PWM 关闭
        return;
    }

    /* --------------------------------------------------------------------------
     * 4. 计算三角函数
     * -------------------------------------------------------------------------- */
    // 优化提示：STM32G4 可以用 CORDIC 硬件加速
    // 为了代码可移植性，先用 math.h，以后替换为 LL_CORDIC_...
    FOC.Sin_Theta = sinf(FOC.Theta);
    FOC.Cos_Theta = cosf(FOC.Theta);

    /* --------------------------------------------------------------------------
     * 5. Park 变换 (2相静止 -> 2相旋转)
     * -------------------------------------------------------------------------- */
    // I_d =  I_alpha * Cos + I_beta * Sin
    // I_q = -I_alpha * Sin + I_beta * Cos
    FOC.I_d =  FOC.I_alpha * FOC.Cos_Theta + FOC.I_beta * FOC.Sin_Theta;
    FOC.I_q = -FOC.I_alpha * FOC.Sin_Theta + FOC.I_beta * FOC.Cos_Theta;

    /* --------------------------------------------------------------------------
     * 6. PID 控制器 (Control Loop)
     * -------------------------------------------------------------------------- */
    // D轴：控制磁通。通常 Id_ref = 0
    // Q轴：控制力矩。Iq_ref = 油门
    if (FOC.State == FOC_STATE_RUN) {
        FOC.V_d = PID_Calc(&PID_Id, FOC.Id_target, FOC.I_d);
        FOC.V_q = PID_Calc(&PID_Iq, FOC.Iq_target, FOC.I_q);
    }

    // [可选] 前馈解耦 / 弱磁控制在这里加入

    /* --------------------------------------------------------------------------
     * 7. 反 Park 变换 (2相旋转 -> 2相静止)
     * -------------------------------------------------------------------------- */
    // V_alpha = V_d * Cos - V_q * Sin
    // V_beta  = V_d * Sin + V_q * Cos
    FOC.V_alpha = FOC.V_d * FOC.Cos_Theta - FOC.V_q * FOC.Sin_Theta;
    FOC.V_beta  = FOC.V_d * FOC.Sin_Theta + FOC.V_q * FOC.Cos_Theta;

    /* --------------------------------------------------------------------------
     * 8. SVPWM 生成 (Output)
     * -------------------------------------------------------------------------- */
    // 如果处于运行状态，更新 PWM
    // 调用 foc_svpwm.c 中的函数，它会处理扇区判断和反向驱动逻辑
    FOC_SVPWM_Update(FOC.V_alpha, FOC.V_beta, FOC.V_bus);
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
