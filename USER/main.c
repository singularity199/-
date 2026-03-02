// 主要头文件
#include "main.h"
#include "stm32g4xx.h"
#include "stm32g4xx_it.h"

// 电机控制
#include "motor_control.h"
#include "dshot.h"

// 硬件控制
#include "adc.h"
#include "tim.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "rcc.h"
#include "dac.h"
#include "FOC.h"
#include "opamp.h"
#include "foc_config.h"

volatile uint16_t g_Throttle = 500;
volatile uint32_t g_AveragePeriod = 0;

// debug
volatile uint32_t debug_isr_cnt = 0;

// debug test
volatile uint16_t debug_CCR1 = 0;
volatile uint16_t debug_CCR2 = 0;
volatile uint16_t debug_CCR3 = 0;
volatile uint16_t debug_throttle = 0;
volatile uint32_t debug_speed_sim = 0;

#define BRIDGE_SELF_TEST_ENABLE 1

static void Bridge_All_Off(void)
{
    TIM_SetPWM(0, 0, 0);
}

static void Bridge_SinglePhase_Pulse(uint8_t phase)
{
    // 先用很小占空比测试桥臂是否正常（约 8%）
    uint16_t amp = (uint16_t)(PWM_PERIOD_ARR * 0.08f);

    uint16_t u = 0, v = 0, w = 0;
    if (phase == 0) u = amp;
    if (phase == 1) v = amp;
    if (phase == 2) w = amp;

    TIM_SetPWM(u, v, w);
}

int main(void) {
    SystemClock_Config();

    // 1. 初始化底层
    PWM_Timer_Init();
    OpAmp_Init();
    ADC_Init();
    LED_Init();
    FOC_Init();

    // 2. 校准零点 (必须在 PWM 开启前!)
    FOC_ADC_CalibrateOffsets();

    // 3. 启动硬件
    PWM_Timer_Start();

#if BRIDGE_SELF_TEST_ENABLE
    // 仅做桥臂硬件自检，不开启FOC ISR链路
    FOC.State = FOC_STATE_IDLE;
    Bridge_All_Off();

    while (1) {
        // U相脉冲
        Bridge_SinglePhase_Pulse(0);
        LL_mDelay(300);
        Bridge_All_Off();
        LL_mDelay(300);

        // V相脉冲
        Bridge_SinglePhase_Pulse(1);
        LL_mDelay(300);
        Bridge_All_Off();
        LL_mDelay(300);

        // W相脉冲
        Bridge_SinglePhase_Pulse(2);
        LL_mDelay(300);
        Bridge_All_Off();
        LL_mDelay(800);
    }
#else
    // 正常FOC路径（当前先保留）
    FOC_ADC_Start();

    // ALIGN 锁轴测试（先不转）
    FOC.State = FOC_STATE_ALIGN;
    FOC.Target_Speed = 0.0f;
    FOC.OpenLoop_Vd = 1.0f;
    FOC.OpenLoop_Vq = 0.0f;
    FOC.Iq_target = 0.0f;
    FOC.Id_target = 0.0f;

    while (1) {
    }
#endif
}
