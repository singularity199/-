/**
  ******************************************************************************
  * @file    foc_opamp.c
  * @brief   FOC 内部运放配置实现
  ******************************************************************************
  */
#include "main.h"
#include "opamp.h"

/**
  * @brief  初始化 GPIO (模拟输入模式)
  */
static void OpAmp_GPIO_Init(void) {
    // 1. 开启 GPIO 时钟 (这是必须的!)
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA1 (OpAmp1_VINP - U相)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA7 (OpAmp2_VINP - V相)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 (OpAmp3_VINP - W相)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  初始化 OpAmp1/2/3
  * @note   配置为 PGA 模式, Gain=16, 内部接地
  */
void OpAmp_Init(void) {
    
    // 1. 初始化 GPIO
    OpAmp_GPIO_Init();
    
    // [修正]: 删除错误的 OpAmp 时钟使能代码
    // OpAmp 是模拟外设，没有 RCC_APB2ENR_OPAMPEN 位。
    // 它的寄存器配置直接有效。
    
    // 建议开启 SYSCFG 时钟 (虽然对于纯 PGA 可能不是强制的，但为了模拟开关稳定性推荐开启)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    /* ======================================================================
     * 配置 OpAmp1 (U相电流)
     * ====================================================================== */
    // 禁用 OpAmp 以便进行配置
    LL_OPAMP_Disable(OPAMP1); 

    // 模式: 独立模式 (Standalone) 
    LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_STANDALONE);

    // 输入选择: 正输入连接到 VINP0 (PA1)
    LL_OPAMP_SetInputNonInverting(OPAMP1, LL_OPAMP_INPUT_NONINVERT_IO0);

    // 负输入配置: PGA模式, 增益 16x, 负端内部接地
    LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputInverting(OPAMP1, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* ======================================================================
     * 配置 OpAmp2 (V相电流)
     * ====================================================================== */
    LL_OPAMP_Disable(OPAMP2);
    LL_OPAMP_SetFunctionalMode(OPAMP2, LL_OPAMP_MODE_STANDALONE);
    
    // 输入选择: 正输入连接到 VINP0 (PA7)
    LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_IO0);
    
    // PGA 设置
    LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputInverting(OPAMP2, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* ======================================================================
     * 配置 OpAmp3 (W相电流)
     * ====================================================================== */
    LL_OPAMP_Disable(OPAMP3);
    LL_OPAMP_SetFunctionalMode(OPAMP3, LL_OPAMP_MODE_STANDALONE);
    
    // 输入选择: 正输入连接到 VINP0 (PB0)
    LL_OPAMP_SetInputNonInverting(OPAMP3, LL_OPAMP_INPUT_NONINVERT_IO0);
    
    // PGA 设置
    LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputInverting(OPAMP3, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* ======================================================================
     * 启动 OpAmps
     * ====================================================================== */
    LL_OPAMP_Enable(OPAMP1);
    LL_OPAMP_Enable(OPAMP2);
    LL_OPAMP_Enable(OPAMP3);

    // 等待启动稳定
    LL_mDelay(1); 
}
