/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/delay.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"

extern volatile unsigned long sys_tick_ms;

int main(void)
{
    SYSCFG_DL_init();       /* 含 TIMER_1MS 初始化 */
    Delay_Init();

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

/* TIMER_1MS 中断 — 1ms 计数 */
void TIMER_1MS_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_1MS_INST)) {
    case DL_TIMER_IIDX_ZERO:
        sys_tick_ms++;
        break;
    default:
        break;
    }
}
