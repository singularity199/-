#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

// --- 硬件与 PWM 参数 ---
#define PWM_PERIOD_ARR      2125 
#define PWM_PRESCALER       0     // <--- 确认这行存在
#define PWM_DEADTIME        120

#define SHUNT_RESISTOR      0.001f
#define OPAMP_GAIN          16.0f

#endif
