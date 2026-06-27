/*
 * delay.c — SysTick 1ms 非阻塞延时
 * 参考 my2024H 工程 clock.c 实现
 */

#include "delay.h"

/* SysTick 中断只能定义一次，这里不重复定义 SysTick_Handler
   留给 empty.c 统一定义 */

volatile unsigned long sys_tick_ms = 0;

/* 初始化 SysTick 为 1ms 周期 */
void Delay_Init(void)
{
    DL_SYSTICK_config(80000);  /* 80MHz / 80000 = 1ms */
}

/* 阻塞延时 ms 毫秒 */
void delay_ms(unsigned int ms)
{
    unsigned long start = sys_tick_ms;
    while (sys_tick_ms - start < ms);
}

/* 获取系统运行毫秒数 */
unsigned long millis(void)
{
    return sys_tick_ms;
}
