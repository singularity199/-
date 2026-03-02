#include "main.h"
#include "dac.h"

void DAC_Init(void){
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

    // 配置 DAC1_CH1
    LL_DAC_SetMode(DAC1, LL_DAC_CHANNEL_1, LL_DAC_MODE_NORMAL_OPERATION);
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
    
    // 等待启动
    LL_mDelay(1);
    
    // 设置输出电压: 
    // V_out = (Data / 4095) * 3.3V
    // 目标: 50mV (对应 50A @ 1mΩ) -> Data ≈ 62
    // 目标: 100mV (对应 100A @ 1mΩ) -> Data ≈ 124
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 62); 
    
    // 触发更新
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);

}

