#include "main.h"
#include "rcc.h"

void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    LL_FLASH_EnablePrefetch();
    // 1. 开启 HSE (8MHz)
    LL_RCC_HSE_Enable();
    while(!LL_RCC_HSE_IsReady());

    // 2. 配置 PLL: Source=HSE, M=2, N=85, R=2
    // 8MHz / 2 * 85 / 2 = 170MHz
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    while(!LL_RCC_PLL_IsReady());

    // 3. 切换系统时钟到 PLL
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    // 4. 配置总线时钟
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1); // HCLK = 170MHz
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);  // PCLK1 = 170MHz
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);  // PCLK2 = 170MHz

    // 5. 设置 SysTick 为 1ms (用于延时函数)
    LL_Init1msTick(170000000);
    LL_SetSystemCoreClock(170000000);
}
