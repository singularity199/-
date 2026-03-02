#include "main.h"
#include "dma.h"

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmAMUX.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cortex.h"

/*
  *DMA 配置（最终决定版）

 * - DMA1 Channel1: ADC1 规则组 DMA
 *     采 OPAMP1 输出 -> Ia
 *    g_adc1_dma_buf[1]

 * - DMA1 Channel2: ADC2 规则组 DMA
 *     采 OPAMP2 / OPAMP3 输出 -> Ib / Ic
 *     g_adc2_dma_buf[2]
 *
 * - DMA1 Channel3: 不使用、不配置、不使能 NVIC
 *   前提：工程中没有其它模块启用 DMA1 Channel3
*/

volatile uint16_t g_adc1_dma_buf[1] = {0};  /* ADC1: Ia */
volatile uint16_t g_adc2_dma_buf[2] = {0};  /* ADC2: Ib, Ic */

void DMA_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

    /* =========================
        DMA1 Channel1 <- ADC1 (Ia)
       ========================= */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
                            LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)g_adc1_dma_buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);

    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

    NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    /* =========================
        DMA1 Channel2 <- ADC2 (Ib/Ic)
       ========================= */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_ADC2);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2,
                            LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)g_adc2_dma_buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);

    LL_DMA_ClearFlag_GI2(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

    NVIC_SetPriority(DMA1_Channel2_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}
