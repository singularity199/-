#include "main.h"
#include "comp.h"
/**
 * @brief  COMP Initialization Function
 * @note   功能: 硬件过流保护 (OCP)
 * 输入+: OPAMP1/2/3 输出 (内部直连)
 * 输入-: 1/2 VREFINT (约 0.6V 阈值)
 * 输出: 连接到 TIM1 BKIN2 (刹车)
 */
void COMP_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. 开启相关时钟 */
    // 核心修正：移除 LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_OPAMP);

    // 开启 GPIO 时钟
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // 2. 配置 GPIO (模拟模式)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

    // PA1 (OpAmp1 Vin+), PA7 (OpAmp2 Vin+)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 (OpAmp3 Vin+)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ========================================================== */
    /* OPAMP1 配置 (A相电流)                                      */
    /* ========================================================== */
    // G431 的 OPAMP 寄存器可以直接访问

    LL_OPAMP_SetPowerMode(OPAMP1, LL_OPAMP_POWERMODE_HIGHSPEED);
    LL_OPAMP_SetFunctionalMode(OPAMP1, LL_OPAMP_MODE_PGA);
    LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);

    // PA1 -> IO0
    LL_OPAMP_SetInputNonInverting(OPAMP1, LL_OPAMP_INPUT_NONINVERT_IO0);
    LL_OPAMP_SetInputInverting(OPAMP1, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* ========================================================== */
    /* OPAMP2 配置 (B相电流)                                      */
    /* ========================================================== */
    LL_OPAMP_SetPowerMode(OPAMP2, LL_OPAMP_POWERMODE_HIGHSPEED);
    LL_OPAMP_SetFunctionalMode(OPAMP2, LL_OPAMP_MODE_PGA);
    LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);

    // PA7 -> IO0
    LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_IO0);
    LL_OPAMP_SetInputInverting(OPAMP2, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* ========================================================== */
    /* OPAMP3 配置 (C相电流)                                      */
    /* ========================================================== */
    LL_OPAMP_SetPowerMode(OPAMP3, LL_OPAMP_POWERMODE_HIGHSPEED);
    LL_OPAMP_SetFunctionalMode(OPAMP3, LL_OPAMP_MODE_PGA);
    LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);

    // PB0 -> IO0
    LL_OPAMP_SetInputNonInverting(OPAMP3, LL_OPAMP_INPUT_NONINVERT_IO0);
    LL_OPAMP_SetInputInverting(OPAMP3, LL_OPAMP_INPUT_INVERT_CONNECT_NO);

    /* 3. 开启 OpAmp */
    LL_OPAMP_Enable(OPAMP1);
    LL_OPAMP_Enable(OPAMP2);
    LL_OPAMP_Enable(OPAMP3);

    /* 4. 等待启动稳定 */
    LL_mDelay(1);
}
