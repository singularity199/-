/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   中断服务函数
  ******************************************************************************
  */

#include "main.h"
#include "stm32g4xx_it.h"

/* 引入必要的头文件 */
#include "FOC.h"      // 核心算法
#include "adc.h"  // 电流零点数据
#include "led.h"      // 状态指示 (避免报错)

/* ==============================================================================
 * ADC1 & ADC2 注入组中断 (JEOC)
 * 注意: ADC1 是 Master，ADC2 是 Slave，我们只开了 ADC1 的中断
 * ============================================================================== */
void ADC1_2_IRQHandler(void)
{
    // 检查是否是注入组转换结束 (JEOC - Injected End of Conversion)
    if(LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        
        // 1. 清除标志位 (必须先清，否则可能重复进中断)
        LL_ADC_ClearFlag_JEOS(ADC1);
        
        // 2. 读取 ADC 原始值 (Raw Data)
        // 硬件连接回顾:
        // U相 -> ADC1_IN13 (Rank1) -> JDR1
        // V相 -> ADC2_IN16 (Rank1) -> JDR1
        // W相 -> ADC2_IN18 (Rank2) -> JDR2 (修正后的G431方案)
        
        int16_t raw_u = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        int16_t raw_v = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        int16_t raw_w = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        // 3. 转换为实际电流 (Amps)
        // 公式: I = (Raw - Offset) * Scale
        // 注意方向: 三电阻低侧采样，ADC读数代表正向电流
        
        FOC.I_u = (float)(raw_u - adc_offsets.offset_u) * CURRENT_SCALE;
        FOC.I_v = (float)(raw_v - adc_offsets.offset_v) * CURRENT_SCALE;
        FOC.I_w = (float)(raw_w - adc_offsets.offset_w) * CURRENT_SCALE;

        // 4. 调用 FOC 核心计算 (耗时必须极短!)
        // 这个函数会计算 PID，SVPWM，并直接更新 TIM1 寄存器
        FOC_Loop_ISR();
        
    }
    
    // 如果开启了规则组中断(EOC)，也要处理，防止死循环
    if(LL_ADC_IsActiveFlag_EOC(ADC1)) {
        LL_ADC_ClearFlag_EOC(ADC1);
    }
}

/* ==============================================================================
 * DMA1 Channel 3 中断 (WS2812 LED)
 * ============================================================================== */
void DMA1_Channel3_IRQHandler(void)
{
    if(LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);
        
        // 停止 DMA 和 TIM16，结束发送
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_TIM_DisableCounter(TIM16);
        LL_TIM_SetCounter(TIM16, 0);
        
        // 清除状态标志 (extern from led.h)
        led_is_transferring = 0; 
    }
}

/* ==============================================================================
 * 系统异常中断 (SysTick 等)
 * ============================================================================== */
void SysTick_Handler(void)
{
    // HAL 库时基 (如果用了 HAL_Delay)
    // HAL_IncTick(); 
    
    // 我们的 LL_mDelay 是基于循环的，其实不需要 SysTick
    // 但如果有其他任务调度，可以在这里加
}
