/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 *
 * 硬件:
 *   UART0 (PA9=TX, PA10=RX) — PC 调试 (CH340)
 *   UART1 (PA8=TX, PA9=RX)   — 四路电机驱动板 (Modbus RTU)
 *   UART3 (PB2=TX, PB3=RX)   — 陀螺仪 (Modbus RTU, RX 中断)
 *
 * SysConfig:
 *   user_INST     = UART0 (Debug)
 *   Motor_INST    = UART1 (电机)
 *   MSPMotor_INST = UART3 (陀螺仪)
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
    Debug_Puts("\r\n=== MSPM0G3507 4-Wheel ===\r\n");

    MotorCtrl_Init();
    delay_ms(100);
    MotorCtrl_Start();
    delay_ms(100);
    Debug_Puts("[Motor] started\r\n");

    LinePatrol_Init();
    Debug_Puts("[Sensor] ready\r\n");

    /* 主循环: 以速度 10 循迹 */
    while (1) {
        delay_ms(5);
        LinePatrol_Track(10);
    }
}

void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_IIDX_RX:
        DL_UART_Main_receiveData(MSPMotor_INST);
        break;
    default:
        break;
    }
}

void SysTick_Handler(void)
{
    if (delay_times != 0) delay_times--;
}
