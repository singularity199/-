/**
  ******************************************************************************
  * @file    foc_adc.h
  * @brief   FOC 电流采样 ADC 驱动头文件 (针对 STM32G431)
  * @author  Gemini FOC Assistant
  * @note    使用注入组 (Injected) + TIM1 TRGO 触发
  ******************************************************************************
  */

#ifndef _ADC_H
#define _ADC_H
#include "main.h"

/* ==============================================================================
 * 硬件参数定义 (根据你的硬件修改)
 * ============================================================================== */

// 采样电阻 (单位: 欧姆)
#define SHUNT_RESISTOR      0.001f  

// 运放增益 (STM32G4 内部 PGA 设置为 16x)
#define OPAMP_GAIN          16.0f   

// ADC 参考电压
#define ADC_VREF            3.3f    

// ADC 分辨率 (12-bit)
#define ADC_RESOLUTION      4096.0f 

/* * 电流标幺化系数 (Amps per ADC Count)
 * 计算公式: I = (ADC_Val / 4096 * 3.3) / (Gain * R_shunt)
 * 简化为乘法: I = ADC_Val * CURRENT_SCALE
 */
#define CURRENT_SCALE       (ADC_VREF / (ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTOR))

/* ==============================================================================
 * 零点偏置 (Offset) - 上电校准值
 * ============================================================================== */
typedef struct {
    uint16_t offset_u; // ADC1 零点
    uint16_t offset_v; // ADC2 零点
    uint16_t offset_w; // ADC3 零点
} CurrentOffsets_t;

extern CurrentOffsets_t adc_offsets;

/* ==============================================================================
 * 函数声明
 * ============================================================================== */

// ADC 硬件初始化 (时钟, 校准, 触发源)
void ADC_Init(void);

// 启动 ADC 注入转换 (使能触发)
void FOC_ADC_Start(void);

// 执行电流零点校准 (需要在 PWM 开启前调用)
void FOC_ADC_CalibrateOffsets(void);

#endif /* __FOC_ADC_H */
