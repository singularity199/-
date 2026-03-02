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

    // 4. ALIGN 锁轴测试（先不转）
    FOC.State = FOC_STATE_ALIGN;
    FOC.Target_Speed = 0.0f;

    // 固定电压指令：先从很小值开始，逐步增加
    // 推荐: Vd=0.6 -> 1.0 -> 1.4，Vq保持0
    FOC.OpenLoop_Vd = 1.0f;
    FOC.OpenLoop_Vq = 0.0f; // 会在控制层被限幅到 ±OPENLOOP_VOLTAGE_LIMIT

    // RUN模式才使用PID目标
    FOC.Iq_target = 0.0f;
    FOC.Id_target = 0.0f;

    while (1) {
        // LED 闪烁证明系统活着
        // static int i = 0;
        // LED_SetColor(0, 0, (i++%20 < 10)?50:0, 0); // 绿色闪烁
        // LED_Show();
        // LL_mDelay(100);
    }
}

