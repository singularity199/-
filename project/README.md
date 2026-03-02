# ESC 项目（STM32G431）

## 概述
本项目以 STM32G431 作为电子调速器（ESC）主控，采用同步整流驱动栅极芯片驱动 MOSFET，低侧电流检测，DMA 搬运数据，速度环/电流环双环 PID，CAN 总线通信，WS2812 显示状态，外部 8 MHz 晶振，电池电压电阻分压采样，热敏电阻反馈温度信息。

## 关键特性
- 主控：STM32G431
- 功率级：同步整流栅极驱动 + MOSFET
- 电流检测：低侧采样
- 数据搬运：DMA
- 控制：速度环 + 电流环双环 PID
- 通信：CAN 总线
- 状态指示：WS2812 LED
- 时钟：外部 8 MHz 晶振
- 电压采样：电池分压
- 温度采样：NTC 热敏电阻反馈

## 硬件接口（基于当前代码）
### PWM/栅极驱动（TIM1）
- 主通道（LIN）：PA8/PA9/PA10 → TIM1_CH1/CH2/CH3（AF6）
- 互补通道（HIN）：PB13/PB14/PB15 → TIM1_CH1N/CH2N/CH3N（AF6）
- ADC 触发：TIM1_CH4 仅作内部触发源，TRGO2 输出给 ADC（无外部引脚）

### 电流采样/模拟前端
- 模拟输入：PA1（OpAmp1 Vin+）、PA7（OpAmp2 Vin+）、PB0（OpAmp3 Vin+）
- OPAMP：3 路 PGA（Gain=16），输出内部连接到 ADC/COMP
- COMP：过流保护输出接入 TIM1 BKIN2（内部链路）

### ADC（规则组 + DMA）
- 触发：TIM1_TRGO2
- 序列：CH3 / CH7 / CH8（3 ranks）
- 采样时间：12.5 cycles（ADC_SAMPLINGTIME_12CYCLES_5）

### 指示与接口
- 状态指示灯：PB9（GPIO 输出）
- DShot 输入：声明为 PA15（仅在头文件声明，源文件实现为空）

## 待在代码中显式配置的功能
- CAN 总线、WS2812 驱动
- 电池电压分压采样、NTC 温度采样
