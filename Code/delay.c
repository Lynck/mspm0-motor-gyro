/*
 * delay.c — SysTick 1ms 非阻塞延时
 * 参考 my2024H 工程 clock.c / SysTick_Init
 *
 * ⚠️ DL_SYSTICK_config(80000) 不会自动开 NVIC 中断
 *    需要手动 NVIC_EnableIRQ + 设 SysTick->CTRL
 */

#include "delay.h"

volatile unsigned long sys_tick_ms = 0;

void Delay_Init(void)
{
    /* 设置 SysTick 周期: 80MHz / 80000 = 1ms */
    DL_SYSTICK_config(80000);

    /* 确保 SysTick 异常使能 (DL_SYSTICK_config 不一定开) */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

void delay_ms(unsigned int ms)
{
    unsigned long start = sys_tick_ms;
    while (sys_tick_ms - start < ms);
}

unsigned long millis(void)
{
    return sys_tick_ms;
}
