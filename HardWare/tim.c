/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM1 PWM 初始化与配置实现
  ******************************************************************************
  */
#include "main.h"
#include "tim.h"
#include "foc_config.h"
/**
  * @brief  配置 TIM1 GPIO (PWM 输出)
  * @note   PA8/9/10 (CH1-3) -> LIN (Low Side)
  * PB13/14/15 (CH1N-3N) -> HIN (High Side)
  */
static void TIM1_GPIO_Init(void) {
    // 开启 GPIO 时钟
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. 配置 PA8, PA9, PA10 (TIM1_CH1, CH2, CH3) */
    // 这些脚接的是 LIN (下管控制)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; // 必须高速，保证波形陡峭
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO; // 外部驱动通常有下拉，这里悬空或下拉
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // G431: PA8/9/10 -> AF6
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 2. 配置 PB13, PB14 (TIM1_CH1N, CH2N) */
    // 这些脚接的是 HIN (上管控制)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // G431: PB13/14 -> AF6
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 3. 配置 PB15 (TIM1_CH3N) - 特殊 AF */
    // 警告: PB15 在 G431 上通常是 AF4，但也可能是 AF6，需查阅具体型号数据手册。
    // 在 STM32G431xB 数据手册中，PB15 连接 TIM1_CH3N 是 AF4 !
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4; // [关键] 务必确认原理图和手册
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  初始化 TIM1
  * @note   Center Aligned Mode 1, PWM Mode 1, Deadtime Enabled
  */
void PWM_Timer_Init(void) {
    
    // 1. 初始化 GPIO
    TIM1_GPIO_Init();

    // 2. 开启 TIM1 时钟
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* ======================================================================
     * 时基配置 (Time Base)
     * ====================================================================== */
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    TIM_InitStruct.Prescaler = PWM_PRESCALER;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP; // Center-aligned mode 1
    TIM_InitStruct.Autoreload = PWM_PERIOD_ARR; 
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    
    // [关键] Repetition Counter (RCR) = 1
    // 效果: 每 2 次溢出 (Underflow & Overflow) 才产生一次 Update Event。
    // 在 Center Mode 1 下，这保证了 Update Event 只在 Underflow (0) 时发生。
    // 从而只在下管导通中心触发 ADC。
    TIM_InitStruct.RepetitionCounter = 1; 
    
    LL_TIM_Init(TIM1, &TIM_InitStruct);

    /* ======================================================================
     * 输出通道配置 (OC Channels)
     * ====================================================================== */
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

    // 使用 PWM Mode 1
    // CNT < CCR -> Output High.
    // 在 CNT=0 时 (采样点)，CNT < CCR 恒成立 -> 输出 High -> 下管导通 -> 采样成功。
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;      // CHx (LIN) Enable
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;     // CHxN (HIN) Enable
    
    // 极性配置: High 代表有效 (导通)
    // 除非你的驱动芯片是低电平有效 (Active Low)，否则通常设为 High
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    
    // 空闲状态 (Idle State): 当 PWM 停止或刹车时引脚电平
    // 为安全起见，通常设为 Low (全关)
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    // 配置 CH1
    TIM_OC_InitStruct.CompareValue = 0; // 初始占空比 0
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

    // 配置 CH2
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);

    // 配置 CH3
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

    // 开启预装载寄存器 (Preload) - 必须开启，否则波形会断裂
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    /* ======================================================================
     * 触发输出配置 (TRGO) -> 给 ADC 用
     * ====================================================================== */
    // Master Mode Select: Update
    // 配合 RCR=1，TRGO 信号将在 CNT=0 时产生
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
    
    // 禁用 TRGO2 (我们不需要)
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);

    /* ======================================================================
     * 死区与刹车配置 (BDTR)
     * ====================================================================== */
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
    
    // 开启死区
    // 120 -> 约 700ns
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE; // 运行模式下关闭状态控制
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE; // 空闲模式下关闭状态控制
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF; 
    TIM_BDTRInitStruct.DeadTime = PWM_DEADTIME;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE; // 暂时不配置硬件刹车引脚
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_LOW;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE; // 需要软件开启主输出
    
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
}
/**
  * @brief  启动 TIM1 输出 (PWM + TRGO)
  */
void PWM_Timer_Start(void) {
    // 1. 确保计数器从 0 开始
    LL_TIM_SetCounter(TIM1, 0);
    
    // 2. 开启主输出 (MOE - Main Output Enable)
    // 这是高级定时器的总开关，不打开这个，任何引脚都不会有输出
    LL_TIM_EnableAllOutputs(TIM1);
    
    // 3. 开启计数器
    LL_TIM_EnableCounter(TIM1);
    
    // 此时 PWM 开始发波，ADC 开始被 TRGO 触发
}

/**
  * @brief  停止 TIM1 输出
  */
void PWM_Timer_Stop(void) {
    // 关闭主输出，所有 MOS 管立即根据 IdleState 关闭 (Low)
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_DisableCounter(TIM1);
}
