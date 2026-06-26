/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"

volatile unsigned int delay_times = 0;

void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

int main(void)
{
    SYSCFG_DL_init();

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

void SysTick_Handler(void)
{
    if (delay_times != 0) delay_times--;
}
