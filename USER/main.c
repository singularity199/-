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
// 驱动数学

// ... SystemClock_Config ...

volatile uint16_t g_Throttle = 500;
volatile uint32_t g_AveragePeriod = 0;

// debug
volatile uint32_t debug_isr_cnt = 0; // 必须加 volatile

// debug test
volatile uint16_t debug_CCR1 = 0;      // 监视 U相 占空比寄存器
volatile uint16_t debug_CCR2 = 0;      // 监视 V相 占空比寄存器
volatile uint16_t debug_CCR3 = 0;      // 监视 W相 占空比寄存器
volatile uint16_t debug_throttle = 0;  // 监视当前油门大小
volatile uint32_t debug_speed_sim = 0; // 模拟的速度增量

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
    PWM_Timer_Start();     // PWM 发波 (此时占空比为0)
    FOC_ADC_Start();  // ADC 等待 TRGO 触发

    // 4. 启动流程: 先 ALIGN，再自动切到开环
    FOC.State = FOC_STATE_ALIGN;

    // 开环目标速度(电角速度, RPM)：从低速爬升到该值
    FOC.Target_Speed = 300.0f;

    // 若需切回开环电流环验证，把 OpenLoop_UseVq 置 0
    // FOC.OpenLoop_UseVq = 0;
    // FOC.Iq_target = 0.4f;
    // FOC.Id_target = 0.0f;

    while (1) {
        // LED 闪烁证明系统活着
        // static int i = 0;
        // LED_SetColor(0, 0, (i++%20 < 10)?50:0, 0); // 绿色闪烁
        // LED_Show();
        // LL_mDelay(100);
    }
}

