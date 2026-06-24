/*
 * debug.c ? UART0 ???? (PA10=TX, PA11=RX)
 * ????? UART0???? SysConfig ????
 */

#include "debug.h"

/* ?? UART ?? */
#define DBG_UART  UART0

/* --------------------------------------------------
 * Debug_Init ? ????? UART0 ????
 * ??: PA10=TX, PA11=RX
 * ??: 115200 bps, 8N1, MFCLK=4MHz
 * -------------------------------------------------- */
void Debug_Init(void)
{
    /* ?? UART0 ????? */
    DL_UART_Main_enablePower(DBG_UART);
    DL_UART_Main_reset(DBG_UART);

    /* ????: PA10 ? UART0_TX, PA11 ? UART0_RX */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);  /* PA10 */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);   /* PA11 */

    /* ????: MFCLK, ??? */
    DL_UART_Main_setClockConfig(DBG_UART, &(DL_UART_Main_ClockConfig){
        .clockSel   = DL_UART_MAIN_CLOCK_MFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    });

    /* ??: ????, ?? */
    DL_UART_Main_init(DBG_UART, &(DL_UART_Main_Config){
        .mode      = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX
    });

    /* 16 ???? */
    DL_UART_Main_setOversampling(DBG_UART, DL_UART_OVERSAMPLING_RATE_16X);

    /* ???: 4MHz / 16 / (2 + 11/16) ? 115200 */
    DL_UART_Main_setBaudRateDivisor(DBG_UART, 2, 11);

    /* ?? TX FIFO????????? */
    DL_UART_Main_setTxFIFOThreshold(DBG_UART, DL_UART_TX_FIFO_LEVEL_1_2_FULL);

    /* ?? UART0 */
    DL_UART_Main_enable(DBG_UART);
}

/* --------------------------------------------------
 * Debug_Putc ? ????????
 * -------------------------------------------------- */
void Debug_Putc(uint8_t c)
{
    DL_UART_transmitDataBlocking(DBG_UART, c);
}

/* --------------------------------------------------
 * Debug_Puts ? ???????
 * -------------------------------------------------- */
void Debug_Puts(const char *s)
{
    while (*s) {
        Debug_Putc((uint8_t)*s++);
    }
}

/* --------------------------------------------------
 * Debug_PutDec ? ??????????
 * -------------------------------------------------- */
void Debug_PutDec(int n)
{
    char buf[12];
    int i = 0;
    unsigned int u;

    if (n < 0) {
        Debug_Putc('-');
        u = (unsigned int)(-n);
    } else {
        u = (unsigned int)n;
    }

    do {
        buf[i++] = '0' + (u % 10);
        u /= 10;
    } while (u);

    while (i) {
        Debug_Putc((uint8_t)buf[--i]);
    }
}

/* --------------------------------------------------
 * Debug_PutHex ? ?????????
 * -------------------------------------------------- */
void Debug_PutHex(uint8_t val)
{
    static const char hex[] = "0123456789ABCDEF";
    Debug_Putc((uint8_t)hex[val >> 4]);
    Debug_Putc((uint8_t)hex[val & 0x0F]);
}

/* --------------------------------------------------
 * fputc ??? ? ? printf ??? UART0
 * TI ArmClang ??????????? stdio
 * -------------------------------------------------- */
int fputc(int ch, FILE *f)
{
    DL_UART_transmitDataBlocking(DBG_UART, (uint8_t)ch);
    return ch;
}
