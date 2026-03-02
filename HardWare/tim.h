/**
  ******************************************************************************
  * @file    tim.h
  * @brief   TIM1 PWM 驱动头文件 (针对 STM32G431 反向驱动硬件)
  * @note    硬件连接: TIM1_CHx (PA8/9/10) -> 驱动器 LIN (下管)
  * TIM1_CHxN (PB13/14/15) -> 驱动器 HIN (上管)
  ******************************************************************************
  */
#ifndef TIM_H
#define TIM_H
#include "foc_config.h"
/* ==============================================================================
 * 宏函数：安全设定占空比
 * ==============================================================================
 * @brief  设定三相 PWM 占空比 (处理反向驱动逻辑)
 * @param  u_val: U相上管导通时间 (0 ~ PWM_PERIOD_ARR)
 * @param  v_val: V相上管导通时间
 * @param  w_val: W相上管导通时间
 * @note   由于 CHx 接 LIN (下管)，PWM Mode 1 下:
 * CCR = 下管导通时间 = ARR - 上管导通时间
 */
__STATIC_INLINE void TIM_SetPWM(uint16_t u_val, uint16_t v_val, uint16_t w_val) {
    // 限制最大幅值防止溢出 (虽然上层 SVPWM 应该处理，但底层再锁一次更安全)
    if (u_val > PWM_PERIOD_ARR) u_val = PWM_PERIOD_ARR;
    if (v_val > PWM_PERIOD_ARR) v_val = PWM_PERIOD_ARR;
    if (w_val > PWM_PERIOD_ARR) w_val = PWM_PERIOD_ARR;

    // 执行反向逻辑减法
    LL_TIM_OC_SetCompareCH1(TIM1, PWM_PERIOD_ARR - u_val);
    LL_TIM_OC_SetCompareCH2(TIM1, PWM_PERIOD_ARR - v_val);
    LL_TIM_OC_SetCompareCH3(TIM1, PWM_PERIOD_ARR - w_val);
}

/* ==============================================================================
 * 函数声明
 * ============================================================================== */

void PWM_Timer_Init(void);
void PWM_Timer_Start(void);
void PWM_Timer_Stop(void);

#endif /* __TIM_H */
