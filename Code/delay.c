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
    DL_TimerA_stopCounter(TIMER_1MS_INST);
    DL_TimerA_clearInterruptStatus(TIMER_1MS_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(TIMER_1MS_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    sys_tick_ms = 0;
    NVIC_ClearPendingIRQ(TIMER_1MS_INST_INT_IRQN);
    NVIC_SetPriority(TIMER_1MS_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(TIMER_1MS_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_1MS_INST);
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
