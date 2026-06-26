/*
 * empty.c — 传感器诊断模式 (只读不驱)
 * 每 500ms 输出 S=位图，用串口观察传感器状态是否实时变化
 */

#include "ti_msp_dl_config.h"
#include "Code/debug.h"
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
    Debug_Puts("\r\n=== Sensor Diag ===\r\n");

    while (1) {
        delay_ms(500);

        uint8_t s = LinePatrol_Read();
        Debug_Puts("S=");
        for (int i = 7; i >= 0; i--)
            Debug_PutDec((s >> i) & 1);
        Debug_Puts("\r\n");
    }
}

void SysTick_Handler(void)
{
    if (delay_times != 0) delay_times--;
}
