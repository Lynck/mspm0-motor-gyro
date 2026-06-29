/*
 * delay.c - CPU cycle blocking delay.
 * It does not depend on Timer/SysTick interrupts, so debug halt or a missing
 * timer ISR will not lock the main loop forever.
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

void Delay_Init(void)
{
    /* Keep this API for main.c compatibility. */
}

void delay_ms(unsigned int ms)
{
    while (ms--) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
    }
}

unsigned long millis(void)
{
    return 0;
}
