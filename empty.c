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
    SYSCFG_DL_init();

    /* ????????? UART0 */
    Debug_Init();

    delay_ms(2000);  /* ????????? */

    Debug_Puts("\r\n=== MSPM0G3507 4-Wheel Controller ===\r\n");

    /* ---- ?????? ---- */
    /* ?? UART3 RX ?? (SysConfig ??? MSPMotor_INST ????) */
    NVIC_ClearPendingIRQ(MSPMotor_INST_INT_IRQN);
    NVIC_EnableIRQ(MSPMotor_INST_INT_IRQN);

    /* ????????? */
    Gyro_Init();
    delay_ms(100);
    Debug_Puts("[Gyro]  configured\r\n");

    /* ---- ???????? ---- */
    MotorCtrl_Init();
    delay_ms(200);

    /* ?????? */
    MotorCtrl_Start();
    delay_ms(200);
    Debug_Puts("[Motor] started\r\n");

    Debug_Puts("[System] READY\r\n");
    Debug_Puts("------------------------------\r\n");

    /* ============================================================
     * ???
     *
     * ??: 10ms (100Hz)
     *   ???: ???????
     *   ? 500ms: ???? + ??????
     *
     * ???????????????????:
     *   - ?? gyro_angle (?? 0.01?) ?????
     *   - ?? gyro_dps   (?? 0.01?/s) ??????
     *   - ?? Wheels_SetSpeeds() ? Wheels_DiffDrive() ????
     * ============================================================ */
    int tick = 0;

    while (1) {
        delay_ms(10);  /* 10ms ?? */
        tick++;

        /* ??????? */
        Gyro_RequestData();

        /* ???????????? */
        if (gyro_rx_done) {
            gyro_rx_done = 0;

            /* gyro_angle: ?? (0.01? ??) */
            /* gyro_dps:   ??? (0.01?/s ??) */

            /* ---- ???????????? ---- */
        }

        /* ? 500ms (50 ? 10ms) ?????? + ?? */
        if (tick >= 50) {
            tick = 0;

            /* ??????? */
            Debug_Puts("A=");  Debug_PutDec((int)gyro_angle);
            Debug_Puts("  W="); Debug_PutDec((int)gyro_dps);
            Debug_Puts("\r\n");

            /* ---- ??: ?????????????? ----
               ??? 10? ?? 1 ????, ?? ?100 */
            int16_t turn = gyro_angle / 10;
            if (turn > 100)  turn = 100;
            if (turn < -100) turn = -100;

            /* ????: ??=0, ??=turn (????) */
            Wheels_DiffDrive(0, turn);
        }
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
