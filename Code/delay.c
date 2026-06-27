/*
 * delay.c — SysTick 1ms 非阻塞延时
 * 纯手动配置，不依赖 SysConfig 的 SYSTICK 模块
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

volatile unsigned long sys_tick_ms = 0;

void Delay_Init(void)
{
    /* 禁用 SysTick (确保干净状态) */
    SysTick->CTRL = 0;

    /* 设置重装载值: 80MHz / 80000 = 1000Hz = 1ms */
    SysTick->LOAD = 79999;  /* 计数到0共80000个周期 */
    SysTick->VAL  = 0;      /* 清当前值 */

    /* 设置优先级 */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    /* 使能: CLKSOURCE=CPU时钟, TICKINT=开中断, ENABLE=启动 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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
