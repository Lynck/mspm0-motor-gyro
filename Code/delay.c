/*
 * delay.c — TimerA 硬件定时器 1ms 非阻塞延时
 * 使用 SysConfig TIMER_1MS 模块 (TIMA0)
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

volatile unsigned long sys_tick_ms = 0;

void Delay_Init(void)
{
    /* TIMER_1MS 由 SYSCFG_DL_init() 自动配置并启动, 无需额外操作 */
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
