#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32g4xx.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h" // <--- 确保这里有
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h" // <--- 确保这里有
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_opamp.h"
#include "stdio.h"

/* 项目级参数（建议只放全局一致的） */
#define THROTTLE_MAX        1024
#define ADC_DMA_BUFFER_SIZE 4

#define ALIGN_TIME_MS       200
#define BLANKING_TIME       20

/* 全局系统节拍与调试计数（跨模块常用） */
extern volatile uint32_t system_millis;
extern volatile uint32_t debug_isr_cnt;

#endif

