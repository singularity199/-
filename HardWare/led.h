/**
  ******************************************************************************
  * @file    led.h
  * @brief   WS2812 驱动头文件 (TIM16 + DMA)
  * @note    PB4 -> TIM16_CH1
  ******************************************************************************
  */

#ifndef __LED_H
#define __LED_H

#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "foc_config.h" // 获取系统配置

/*全局变量*/
extern volatile uint8_t led_is_transferring;

/* ==============================================================================
 * WS2812 参数定义
 * ============================================================================== */
#define LED_NUM             1       // LED 数量 (你的电调可能只有一颗灯?)
#define LED_DATA_LEN        (24 * LED_NUM) // 每个灯 24 bit (G R B)
#define LED_RESET_LEN       50      // Reset 信号需要的 "0" 的个数 (50个周期 > 60us)

// 这里的 ARR 取决于 TIM16 的时钟。
// 假设 TIM16 时钟 = 170MHz
// 800kHz PWM -> Period = 170M / 800k = 212.5 -> 取 212
#define WS2812_ARR          212

// 0码与1码的占空比 (CCR值)
#define WS2812_0_CODE       68      // ~32% of 212
#define WS2812_1_CODE       136     // ~64% of 212

/* ==============================================================================
 * 函数声明
 * ============================================================================== */
void LED_Init(void);
void LED_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void LED_Show(void); // 启动 DMA 发送

#endif
