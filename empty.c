/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/motor_ctrl.h"

volatile unsigned int delay_times = 0;

void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

int main(void)
{
    SYSCFG_DL_init();

    MotorCtrl_Init();
    delay_ms(200);
    MotorCtrl_Start();
    delay_ms(200);

    while (1) {
        /* 前进 3 秒 */
        Wheels_SetSpeeds(200, 200, 200, 200);
        delay_ms(3000);

        /* 停止 1 秒 */
        Wheels_SetSpeeds(0, 0, 0, 0);
        delay_ms(1000);

        /* 原地右转 1 秒 */
        Wheels_SetSpeeds(100, 100, -100, -100);
        delay_ms(1000);

        /* 停止 1 秒 */
        Wheels_SetSpeeds(0, 0, 0, 0);
        delay_ms(1000);
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
