/*
 * delay.c — TimerA 硬件定时器 1ms
 * TIMER_1MS (TIMA0), SysConfig 自动配置周期
 * 中断需要手动使能 NVIC + 清除标志
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

volatile unsigned long sys_tick_ms = 0;

void Delay_Init(void)
{
    /* TIMER_1MS 时钟和周期已在 SYSCFG_DL_init() 中配置
       但中断需要手动开 */
    DL_TimerA_clearInterruptStatus(TIMER_1MS_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_1MS_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1MS_INST_INT_IRQN);
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
