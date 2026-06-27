/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/delay.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"

/* SysTick 全局计数器 (在 delay.c 中定义) */
extern volatile unsigned long sys_tick_ms;

int main(void)
{
    SYSCFG_DL_init();
    Delay_Init();  /* 手动初始化 SysTick 1ms */

    Debug_Init();
    delay_ms(500);
    Debug_Puts("\r\n=== MSPM0G3507 Line Patrol ===\r\n");

    MotorCtrl_Init();
    delay_ms(100);
    MotorCtrl_Start();
    delay_ms(100);
    Debug_Puts("[Motor] started\r\n");

    LinePatrol_Init();
    Debug_Puts("[Sensor] ready\r\n");

    while (1) {
        delay_ms(5);
        LinePatrol_Track(10);
    }
}

/* SysTick 中断 — 1ms 计数 */
void SysTick_Handler(void)
{
    sys_tick_ms++;
}
