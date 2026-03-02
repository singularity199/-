/**
 ******************************************************************************
 * @file    foc_adc.c
 * @brief   FOC ADC 驱动实现 (修正版: 针对 G431CBT6 无 ADC3 的情况)
 * @note    ADC1 处理 U相; ADC2 处理 V相 和 W相 (双通道扫描)
 ******************************************************************************
 */
#include "adc.h"
#include "main.h"

CurrentOffsets_t adc_offsets = {2048, 2048, 2048};

static void ADC_EnableRegulator(ADC_TypeDef *ADCx)
{
    if (LL_ADC_IsInternalRegulatorEnabled(ADCx) == 0)
    {
        LL_ADC_DisableDeepPowerDown(ADCx); //
        LL_ADC_EnableInternalRegulator(ADCx);
        LL_mDelay(1);
    }
}

void ADC_Init(void)
{
    /* 1. 开启时钟 ========================================================== */
    // G431 只有 ADC12，没有 ADC345
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    // 设置 ADC12 时钟源
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    /* 2. 稳压器与自校准 ==================================================== */
    ADC_TypeDef *adcs[] = {ADC1, ADC2};
    int i;
    for (i = 0; i < 2; i++)
    {
        ADC_EnableRegulator(adcs[i]);
        LL_ADC_StartCalibration(adcs[i], LL_ADC_SINGLE_ENDED);
        while (LL_ADC_IsCalibrationOnGoing(adcs[i]) != 0)
            ;
    }

    /* 3. 配置 ADC1 (Master) - U相 ========================================== */
    // U相 -> OpAmp1 -> ADC1_IN13

    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    // ADC1 只采一个通道 (U)
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_13);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_6CYCLES_5);

    /* 4. 配置 ADC2 (Slave) - V相 & W相 ===================================== */
    // V相 -> OpAmp2 -> ADC2_IN16 (Rank 1)
    // W相 -> OpAmp3 -> ADC2_IN18 (Rank 2) -> 修正点！

    LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);
    // ADC2 也配置为 TIM1 TRGO 触发，确保同步
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    // ADC2 需要扫描 2 个通道
    LL_ADC_INJ_SetSequencerLength(ADC2, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);

    // Rank 1: V相 (OpAmp2)
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_16);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_6CYCLES_5);

    // Rank 2: W相 (OpAmp3) - 注意这里使用了 ADC2_IN18
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_18);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_18, LL_ADC_SAMPLINGTIME_6CYCLES_5);

    /* 5. 中断配置 ========================================================== */
    // 依然只开 ADC1 的中断。
    // 虽然 ADC2 转换两个通道比 ADC1 慢一点点，但 JEOC 中断发生在 ADC1 完成时。
    // 为了保险，我们可以在 ISR 里稍微等一下 ADC2，或者开启 ADC2 的中断。
    // 最好的做法：使用 ADC1 的 JEOC，但在读取时确认 ADC2 也就绪。
    // 由于 ADC 是并行启动的，Rank 1 是同步的，Rank 2 会晚 12 个时钟，通常进中断时肯定好了。
    LL_ADC_EnableIT_JEOS(ADC1);

    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

void FOC_ADC_Start(void)
{
    ADC_TypeDef *adcs[] = {ADC1, ADC2};

    for (int i = 0; i < 2; i++)
    {
        ADC_TypeDef *a = adcs[i];

        if ((a->CR & ADC_CR_ADEN) == 0)   // 只有未使能才走 ADRDY 流程
        {
            LL_ADC_ClearFlag_ADRDY(a);
            LL_ADC_Enable(a);
            while (LL_ADC_IsActiveFlag_ADRDY(a) == 0) {}
        }

        LL_ADC_INJ_StartConversion(a); // Arm trigger
    }
}


void FOC_ADC_CalibrateOffsets(void)
{
    const int calib_samples = 1000;
    int32_t sum_u = 0, sum_v = 0, sum_w = 0;

    // ==========================================
    // 关键修正 1: 临时关闭 ADC 全局中断
    // 防止 ISR 抢在 while 循环之前清除 JEOS 标志
    // ==========================================
    NVIC_DisableIRQ(ADC1_2_IRQn); 

    // 0) 确保不在校准中
    while (ADC1->CR & ADC_CR_ADCAL) {}
    while (ADC2->CR & ADC_CR_ADCAL) {}

    // 1) 停止注入转换
    LL_ADC_INJ_StopConversion(ADC1);
    LL_ADC_INJ_StopConversion(ADC2);
    while (ADC1->CR & ADC_CR_JADSTART) {}
    while (ADC2->CR & ADC_CR_JADSTART) {}

    // 2) 切软件触发 + 关外触发边沿
    ADC1->JSQR &= ~ADC_JSQR_JEXTEN;
    ADC2->JSQR &= ~ADC_JSQR_JEXTEN;

    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_SOFTWARE);

    // 3) 确保 ADC 已使能且 ready
    if (!(ADC1->CR & ADC_CR_ADEN)) LL_ADC_Enable(ADC1);
    if (!(ADC2->CR & ADC_CR_ADEN)) LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) {}

    for (int i = 0; i < calib_samples; i++)
    {
        // 4) 清 flags (清除之前的状态)
        LL_ADC_ClearFlag_JEOC(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOC(ADC2);
        LL_ADC_ClearFlag_JEOS(ADC2);

        // 5) 软件触发
        ADC1->CR |= ADC_CR_JADSTART;
        volatile uint32_t TEST =  ADC1->CR;
        volatile uint32_t debug_JSQR = ADC1->JSQR;
        ADC2->CR |= ADC_CR_JADSTART;

        // 6) 等 JEOS (现在中断关了，这里肯定能等到了)
        uint32_t t = 1000000;
        while (!(ADC1->ISR & ADC_ISR_JEOS) && --t) {}
        
        // 增加超时处理，避免死循环
        if (t == 0) {
            // 超时错误处理，比如 break 或者记录错误
            break; 
        }

        t = 1000000;
        while (!(ADC2->ISR & ADC_ISR_JEOS) && --t) {}
        if (t == 0) break;

        // 7) 读数
        sum_u += (int32_t)ADC1->JDR1;
        sum_v += (int32_t)ADC2->JDR1; 
        sum_w += (int32_t)ADC2->JDR2; 
    }

    adc_offsets.offset_u = (uint16_t)(sum_u / calib_samples);
    adc_offsets.offset_v = (uint16_t)(sum_v / calib_samples);
    adc_offsets.offset_w = (uint16_t)(sum_w / calib_samples);

    // 8) 恢复现场
    LL_ADC_INJ_StopConversion(ADC1);
    LL_ADC_INJ_StopConversion(ADC2);
    
    // 恢复 Trigger Source
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    // 清除由于手动触发可能残留的任何中断标志
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);

    // ==========================================
    // 关键修正 2: 恢复 ADC 中断
    // ==========================================
    NVIC_EnableIRQ(ADC1_2_IRQn); 

    LL_ADC_INJ_StartConversion(ADC1); // arm
    LL_ADC_INJ_StartConversion(ADC2); // arm
}
