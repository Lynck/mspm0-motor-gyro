/*
 * empty.c ? MSPM0G3507 ??????????
 *
 * ????:
 *   UART0 (PA10=TX, PA11=RX) ? ???? (??CH340, PC printf)
 *   UART1 (PA8=TX, PA9=RX)   ? ??????? (Modbus RTU ??)
 *   UART3 (PB2=TX, PB3=RX)   ? ??? (Modbus RTU ????/???)
 *
 * SysConfig ??:
 *   user_INST    = UART1 (PA8/PA9, ???)
 *   MSPMotor_INST = UART3 (PB2/PB3, ???)
 *
 * ?????:
 *   Code/crc16.h/c       ? Modbus CRC16 ?? (???)
 *   Code/debug.h/c        ? UART0 ???? (?????)
 *   Code/motor_ctrl.h/c   ? ???? (Wheels_SetSpeeds/Wheels_DiffDrive)
 *   Code/gyro.h/c         ? ??????? (Gyro_RequestData/Gyro_ParseByte)
 */

#include "ti_msp_dl_config.h"
#include "Code/crc16.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/gyro.h"
#include "line_patrol.h"

/* SysTick ???? (? 1ms ? 1) */
volatile unsigned int delay_times = 0;

/* ---- ???? (ms ?) ---- */
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

/* ================================================================
 * main ? ????
 * ================================================================ */
int main(void)
{
    /* SysConfig ?????:
       - UART1 (user_INST): PA8=TX, PA9=RX, 115200 8N1
       - UART3 (MSPMotor_INST): PB2=TX, PB3=RX, 115200 8N1, RX??
       - SysTick: 1ms (80MHz / 80000)
       - ??: SYSPLL 80MHz */
    SYSCFG_DL_init();       // SysConfig 初始化（含GPIO + 时钟）
    
    MotorCtrl_Init();       // 电机驱动初始化
    MotorCtrl_Start();      // 启动电机
    LinePatrol_Init();      // 灰度传感器初始化

    while (1) {
        delay_ms(5);                    // 5ms 一次
        LinePatrol_Track(200);          // 以速度 200 循迹，PID 自动转向
    }
}

/* ================================================================
 * UART3 RX ?????? ? ???????
 *
 * SysConfig ??: MSPMotor_INST_IRQHandler ? UART3_INT_IRQn
 * ???: 0 (??)
 *
 * ?????????? Gyro_ParseByte() ?????
 * ??? (9??): 0A 03 04 AH AL WH WL CRCL CRCH
 * ================================================================ */
void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_IIDX_RX:
        /* ???????????????? */
        Gyro_ParseByte(DL_UART_Main_receiveData(MSPMotor_INST));
        break;
    default:
        break;
    }
}

/* ================================================================
 * SysTick ????? ? 1ms ??
 *
 * SysConfig ??:
 *   period  = 80000 (80MHz / 80000 = 1kHz)
 *   ??? = 3
 *
 * ?? delay_ms() ????
 * ================================================================ */
void SysTick_Handler(void)
{
    if (delay_times != 0) {
        delay_times--;
    }
}
