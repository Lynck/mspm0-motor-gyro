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
 *
 * 模块依赖:
 *   Code/crc16.h/c       — Modbus CRC16
 *   Code/debug.h/c        — 调试串口
 *   Code/motor_ctrl.h/c   — 电机控制 (Wheels_SetSpeeds / Wheels_DiffDrive)
 *   Code/gyro.h/c         — 陀螺仪
 *   Code/line_patrol.h/c  — 八路灰度循迹 + PID
 */

#include "ti_msp_dl_config.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"

/* SysTick 计数 (每 1ms 减 1) */
volatile unsigned int delay_times = 0;

/* 延时函数 (ms) */
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

/* ================================================================
 * main
 * ================================================================ */
int main(void)
{
    /* SysConfig 初始化: UART0/1/3 + GPIO + SysTick + 时钟 */
    SYSCFG_DL_init();

    /* 调试串口初始化 */
    Debug_Init();
    delay_ms(500);
    Debug_Puts("\r\n=== MSPM0G3507 4-Wheel Controller ===\r\n");
    Debug_Puts("[Init] done\r\n");

    /* 电机初始化 */
    MotorCtrl_Init();
    delay_ms(100);
    MotorCtrl_Start();
    delay_ms(100);
    Debug_Puts("[Motor] started\r\n");

    /* 灰度传感器初始化 */
    LinePatrol_Init();
    Debug_Puts("[Sensor] ready\r\n");

    Debug_Puts("[System] entering loop...\r\n");

    /* 主循环 — 先简单测试电机 */
    int cnt = 0;
    while (1) {
        delay_ms(10);
        cnt++;

        /* 前 2 秒: 直走 100，确认电机能动 */
        if (cnt < 200) {
            Wheels_SetSpeeds(100, 100, 100, 100);
            if (cnt == 100) Debug_Puts("[Test] forward 100\r\n");
        }
        /* 之后: 停止 */
        else {
            Wheels_SetSpeeds(0, 0, 0, 0);
        }

        /* 每秒输出一次 */
        if (cnt % 100 == 0) {
            Debug_Puts("tick=");
            Debug_PutDec(cnt);
            Debug_Puts("\r\n");
        }
    }
}

/* ================================================================
 * UART3 RX 中断 — 陀螺仪数据接收
 * ================================================================ */
void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_IIDX_RX:
        /* Gyro_ParseByte(DL_UART_Main_receiveData(MSPMotor_INST)); */
        break;
    default:
        break;
    }
}

/* ================================================================
 * SysTick 中断 — 1ms 定时
 * ================================================================ */
void SysTick_Handler(void)
{
    if (delay_times != 0) {
        delay_times--;
    }
}
