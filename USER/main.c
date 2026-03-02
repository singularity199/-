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

    // 4. 进入简单的开环测试 (绕过 motor_control)
    FOC.State = FOC_STATE_OPEN_LOOP;
    FOC.Target_Speed = 50.0f; // 慢速旋转
    FOC.Iq_target = 0.5f;     // 给一点微小的电流 (0.5A)
    FOC.Id_target = 0.0f;

    while (1) {
        // LED 闪烁证明系统活着
        // static int i = 0;
        // LED_SetColor(0, 0, (i++%20 < 10)?50:0, 0); // 绿色闪烁
        // LED_Show();
        // LL_mDelay(100);
    }
}

