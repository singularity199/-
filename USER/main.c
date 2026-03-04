#include "main.h"
#include "stm32g4xx.h"
#include "stm32g4xx_it.h"

#include "motor_control.h"
#include "dshot.h"

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
volatile uint32_t debug_isr_cnt = 0;

volatile uint16_t debug_CCR1 = 0;
volatile uint16_t debug_CCR2 = 0;
volatile uint16_t debug_CCR3 = 0;
volatile uint16_t debug_throttle = 0;
volatile uint32_t debug_speed_sim = 0;

// 1: 安全模式（默认，禁止功率输出）
// 0: 进入FOC启动流程测试
#define SAFE_POWER_STAGE_MODE 1

int main(void) {
    SystemClock_Config();

    PWM_Timer_Init();
    OpAmp_Init();
    ADC_Init();
    LED_Init();
    FOC_Init();

    FOC_ADC_CalibrateOffsets();

#if SAFE_POWER_STAGE_MODE
    // 紧急保护：默认不启动PWM输出，不启动FOC ISR链路
    FOC.startup_enable = 0;
    FOC.State = FOC_STATE_IDLE;
    FOC.V_d = 0.0f;
    FOC.V_q = 0.0f;

    // 确保占空比为0并关闭PWM输出
    TIM_SetPWM(0, 0, 0);
    PWM_Timer_Stop();

    while (1) {
        // 可选心跳灯
        // static uint8_t t = 0;
        // LED_SetColor(0, 0, (t++ & 0x10) ? 8 : 0, 0);
        // LED_Show();
        // LL_mDelay(50);
    }
#else
    PWM_Timer_Start();
    FOC_ADC_Start();

    // 完整启动序列参数：IDLE -> ALIGN -> OPEN_LOOP
    FOC.startup_enable = 1;
    FOC.State = FOC_STATE_IDLE;
    FOC.state_counter = 0;

    // 200ms 空闲、1000ms 预定位
    FOC.idle_cycles = (uint32_t)(FOC_CTRL_FREQ * 0.2f);
    FOC.align_cycles = (uint32_t)(FOC_CTRL_FREQ * 1.0f);

    // 开环持续时间（0 表示一直保持开环，不自动切 RUN）
    FOC.openloop_cycles = 0;

    // 预定位电压（锁轴）
    FOC.Align_Vd = 0.6f;
    FOC.Align_Vq = 0.0f;

    // 开环启动参数
    FOC.Target_Speed = 20.0f;     // rad/s
    FOC.OpenLoop_Accel = 40.0f;   // rad/s^2
    FOC.OpenLoop_Vd = 0.0f;
    FOC.OpenLoop_Vq = 0.4f;

    // RUN 模式目标（当前先不用）
    FOC.Iq_target = 0.0f;
    FOC.Id_target = 0.0f;

    while (1) {
    }
#endif
}
