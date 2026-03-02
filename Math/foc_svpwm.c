/**
  ******************************************************************************
  * @file    foc_svpwm.c
  * @brief   SVPWM 算法实现
  ******************************************************************************
  */

#include "foc_svpwm.h"

/**
  * @brief  找出三个数中的最大值和最小值
  * @note   用于计算共模偏移电压 (零序分量)
  */
static void find_min_max(float a, float b, float c, float *min, float *max) {
    *max = a;
    *min = a;
    
    if (b > *max) *max = b;
    if (b < *min) *min = b;
    
    if (c > *max) *max = c;
    if (c < *min) *min = c;
}

/**
  * @brief  SVPWM 更新函数
  * @note   执行时间必须极短，通常在 5us 以内
  */
void FOC_SVPWM_Update(float v_alpha, float v_beta, float v_bus) {
    
    // 1. 反 Clarke 变换 (Inverse Clarke)
    // 将 Alpha/Beta (两相静止) 投影到 a, b, c (三相静止) 轴上
    // 注意：这里的 V_a, V_b, V_c 是纯正弦波，相对于电机中性点
    
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + _SQRT3_DIV_2 * v_beta;
    float v_c = -0.5f * v_alpha - _SQRT3_DIV_2 * v_beta;

    // 2. 零序分量注入 (Min-Max Injection)
    // 这是生成 SVPWM "马鞍波" 的核心魔法。
    // 为了让电压矢量合成后的最大幅值利用率提高 15.47% (相比 SPWM)，
    // 我们需要将三相波形的 "中间点" 挪到 "母线电压的中间"。
    
    float v_min, v_max;
    find_min_max(v_a, v_b, v_c, &v_min, &v_max);
    
    // 计算零序电压 (Common Mode Voltage)
    // V_neutral = -(Max + Min) / 2
    float v_neutral = -0.5f * (v_max + v_min);

    // 叠加零序电压，得到对地输出电压 (Saddle Shape)
    v_a += v_neutral;
    v_b += v_neutral;
    v_c += v_neutral;

    // 3. 计算占空比 (Duty Cycle)
    // 此时 v_a/b/c 的范围大约是 [-Vbus/sqrt(3), +Vbus/sqrt(3)]
    // 我们需要将其映射到 [0, 1] 的占空比范围
    // 公式: Duty = (V_phase / V_bus) + 0.5
    
    float duty_a = (v_a / v_bus) + 0.5f;
    float duty_b = (v_b / v_bus) + 0.5f;
    float duty_c = (v_c / v_bus) + 0.5f;

    // 4. 硬件安全限制 (Clamping for Low-side Sensing)
    // [关键]: 如果占空比接近 100%，下管开启时间太短，ADC 无法采样。
    // 所以必须强制限制最大占空比 (例如 0.95)
    // 同时也限制最小占空比防止负数
    
    if (duty_a > MAX_DUTY_CYCLE_LIMIT) duty_a = MAX_DUTY_CYCLE_LIMIT;
    if (duty_a < 0.01f)                duty_a = 0.01f;

    if (duty_b > MAX_DUTY_CYCLE_LIMIT) duty_b = MAX_DUTY_CYCLE_LIMIT;
    if (duty_b < 0.01f)                duty_b = 0.01f;

    if (duty_c > MAX_DUTY_CYCLE_LIMIT) duty_c = MAX_DUTY_CYCLE_LIMIT;
    if (duty_c < 0.01f)                duty_c = 0.01f;

    // 5. 转换为定时器计数值 (Timer Counts)
    // t_a 代表 "上管导通时间" (高电平时间，在 Mode 1 下)
    uint16_t t_a = (uint16_t)(duty_a * PWM_PERIOD_ARR);
    uint16_t t_b = (uint16_t)(duty_b * PWM_PERIOD_ARR);
    uint16_t t_c = (uint16_t)(duty_c * PWM_PERIOD_ARR);

    // 6. 写入硬件层
    // 调用我们在 tim.h 中定义的宏，它会自动处理 "反向驱动硬件" 的逻辑
    // TIM_SetPWM(t_a, t_b, t_c) -> 内部执行 CCR = ARR - t_x
    TIM_SetPWM(t_a, t_b, t_c);
}
