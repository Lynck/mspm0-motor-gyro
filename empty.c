/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/delay.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"
#include "Code/gyro.h"

#define MOTOR_SAFE_STOP_MODE 1
#define MOTOR_DIAG_MODE 0

int main(void)
{
    SYSCFG_DL_init();
    Delay_Init();

    Debug_Init();
    delay_ms(500);
    Debug_Puts("\r\n=== MSPM0G3507 Line Patrol ===\r\n");

#if MOTOR_SAFE_STOP_MODE
    MotorCtrl_EmergencyStop();
    Debug_Puts("[Motor] safe-stop mode: no start command\r\n");
    while (1) {
        MotorCtrl_EmergencyStop();
        delay_ms(100);
    }
#endif

    MotorCtrl_Init();
    MotorCtrl_Start();
    delay_ms(100);
    Debug_Puts("[Motor] started\r\n");

#if MOTOR_DIAG_MODE
    Debug_Puts("[Motor] diagnostic: one motor at a time\r\n");
    while (1) {
        MotorCtrl_TestSequence(5, 2000);
    }
#endif

    LinePatrol_Init();
    Debug_Puts("[Sensor] ready\r\n");

    while (1) {
        delay_ms(5);
        LinePatrol_Track(10);
    }
}

void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        Gyro_ParseByte(DL_UART_Main_receiveData(MSPMotor_INST));
        break;
    default:
        break;
    }
}
