/**
  ******************************************************************************
  * @file    led.c
  * @brief   WS2812 驱动实现 (PB4, TIM16_CH1, DMA)
  ******************************************************************************
  */

#include "led.h"

// DMA 发送缓冲区
// 包含: [G,R,B 数据 bits] + [Reset 信号的 0]
// 为什么是 uint16_t? 因为我们要往 TIMx->CCR1 (16位寄存器) 里搬数据
uint16_t led_dma_buffer[LED_DATA_LEN + LED_RESET_LEN] = {0};

// 状态标志
volatile uint8_t led_is_transferring = 0;

/**
  * @brief  初始化 TIM16 和 DMA
  */
void LED_Init(void) {
    /* 1. 开启时钟 */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1); // G4 必须开 DMAMUX

    /* 2. 配置 GPIO PB4 -> AF1 (TIM16_CH1) */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1; // PB4 AF1 is TIM16_CH1
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 3. 配置 TIM16 (PWM 模式) */
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler = 0; // 不分频, 170MHz
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = WS2812_ARR - 1; // 800kHz
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM16, &TIM_InitStruct);

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.CompareValue = 0; // 默认输出低
    LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    
    // 开启预装载
    LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);
    
    // 开启 TIM16 的 DMA 请求 (Update Event)
    LL_TIM_EnableDMAReq_UPDATE(TIM16); // 注意: TIM16 通常使用 Update 请求来搬运 CCR

    // 但 TIM16_CH1 也有专门的 CC1 DMA 请求。
    // 为了更准确，通常推荐使用 CC1 DMA 请求搬运 CCR1。
    // 这里我们修正为 CC1 请求：
    LL_TIM_DisableDMAReq_UPDATE(TIM16);
    LL_TIM_EnableDMAReq_CC1(TIM16);

    /* 4. 配置 DMA1 Channel 3 (TIM16_CH1 / UP) */
    // 查表: G4 DMAMUX Request ID for TIM16_CH1 is 78 (LL_DMAMUX_REQ_TIM16_CH1)
    
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_TIM16_CH1);
    
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW); // 低优先级，别抢 FOC
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL); // 发送一次停止
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT); // 寄存器地址不动
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);   // 内存地址递增
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);   // 16-bit CCR
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);   // 16-bit buffer
    
    // 设置 DMA 目标地址: TIM16->CCR1
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&TIM16->CCR1);
    
    // 开启 DMA 传输完成中断 (可选，用于清除标志位)
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 3); // 优先级要比 ADC/TIM1 低！
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    
    /* 5. 初始化缓冲区为全 0 (Reset) */
    for(int i=0; i < LED_DATA_LEN + LED_RESET_LEN; i++) {
        led_dma_buffer[i] = 0;
    }
}

/**
  * @brief  设置 LED 颜色 (填充 Buffer)
  * @note   WS2812 格式是 G-R-B
  */
void LED_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if(index >= LED_NUM) return;
    
    uint32_t start_pos = index * 24;
    
    // 填充 Green (8 bit)
    for(int i=0; i<8; i++) {
        led_dma_buffer[start_pos + i] = (g & (0x80 >> i)) ? WS2812_1_CODE : WS2812_0_CODE;
    }
    // 填充 Red (8 bit)
    for(int i=0; i<8; i++) {
        led_dma_buffer[start_pos + 8 + i] = (r & (0x80 >> i)) ? WS2812_1_CODE : WS2812_0_CODE;
    }
    // 填充 Blue (8 bit)
    for(int i=0; i<8; i++) {
        led_dma_buffer[start_pos + 16 + i] = (b & (0x80 >> i)) ? WS2812_1_CODE : WS2812_0_CODE;
    }
    
    // 后面的 Reset 部分保持为 0，不需要动
}

/**
  * @brief  启动 DMA 刷新显示
  */
void LED_Show(void) {
    if(led_is_transferring) return; // 上一次还没发完，跳过
    
    led_is_transferring = 1;

    // 1. 设置 DMA 数据长度
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)led_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, LED_DATA_LEN + LED_RESET_LEN);
    
    // 2. 清除标志位
    LL_DMA_ClearFlag_TC3(DMA1);
    
    // 3. 开启 DMA
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    
    // 4. 开启 TIM16 计数 (PWM)
    LL_TIM_EnableCounter(TIM16);
    LL_TIM_EnableAllOutputs(TIM16); // MOE
}


