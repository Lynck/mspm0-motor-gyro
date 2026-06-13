/*
 * user_INST = UART1 (PA8=TX, PA9=RX) -> driver board (Modbus master)
 * MSPMotor_INST = UART3 (PB2=TX, PB3=RX) -> gyro (receive angle data)
 * Manual UART0 (PA10=TX, PA11=RX) -> debug PC
 */

#include "ti_msp_dl_config.h"

volatile unsigned int delay_times = 0;
void delay_ms(unsigned int ms);

/* ---------- externs from imu.c ---------- */
extern volatile int16_t gyro_angle_raw;
extern volatile int16_t gyro_dps_raw;
extern volatile uint8_t  gyro_rx_done;
extern void Gyro_ConfigReportRateRx(void);
extern void Gyro_RequestData(void);
extern void Gyro_ParseFrame(uint8_t data);

/* ---------- CRC16 from motor_crc.c ---------- */
extern unsigned short CRC16(uint8_t *p, unsigned short len);

/* =====================================================
 *  Driver board commands  (via user_INST = UART1)
 * ===================================================== */
#define DRV_ADDR  0x0A

void drv_send(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++) {
        while (DL_UART_isBusy(user_INST));
        DL_UART_Main_transmitData(user_INST, buf[i]);
    }
}

void drv_start(void)
{
    uint8_t f[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC8, 0xB3};
    drv_send(f, 8);
}

void drv_stop(void)
{
    uint8_t f[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x00, 0x89, 0x73};
    drv_send(f, 8);
}

void drv_set_speeds(int16_t v0, int16_t v1, int16_t v2, int16_t v3)
{
    uint8_t f[15]; int idx = 0;
    f[idx++] = DRV_ADDR;       f[idx++] = 0x10;
    f[idx++] = 0x00;           f[idx++] = 0x00;
    f[idx++] = 0x00;           f[idx++] = 0x04;
    f[idx++] = 0x08;
    f[idx++] = (v0>>8)&0xFF;   f[idx++] = v0&0xFF;
    f[idx++] = (v1>>8)&0xFF;   f[idx++] = v1&0xFF;
    f[idx++] = (v2>>8)&0xFF;   f[idx++] = v2&0xFF;
    f[idx++] = (v3>>8)&0xFF;   f[idx++] = v3&0xFF;
    uint16_t crc = CRC16(f, idx);
    f[idx++] = crc & 0xFF;     f[idx++] = (crc>>8)&0xFF;
    drv_send(f, idx);
}

/* =====================================================
 *  Debug UART0  (manual init)
 * ===================================================== */
#define DBG_UART  UART0

void dbg_init(void)
{
    /* enable UART0 power & clock */
    DL_UART_Main_enablePower(DBG_UART);
    DL_UART_Main_reset(DBG_UART);

    /* pinmux: PA10=TX, PA11=RX */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);

    /* config UART0: 115200 8N1, MFCLK=4MHz */
    DL_UART_Main_setClockConfig(DBG_UART, &(DL_UART_Main_ClockConfig){
        .clockSel = DL_UART_MAIN_CLOCK_MFCLK, .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1});
    DL_UART_Main_init(DBG_UART, &(DL_UART_Main_Config){
        .mode = DL_UART_MAIN_MODE_NORMAL, .direction = DL_UART_MAIN_DIRECTION_TX_RX});
    DL_UART_Main_setOversampling(DBG_UART, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(DBG_UART, 2, 11);  /* 4MHz / 16 / (2+11/16) ~= 115107 */
    DL_UART_Main_enable(DBG_UART);
}

void dbg_putc(uint8_t c)
{
    DL_UART_transmitDataBlocking(DBG_UART, c);
}

void dbg_puts(const char *s)
{
    while (*s) dbg_putc(*s++);
}

void dbg_putdec(int n)
{
    char b[12]; int i=0; unsigned u;
    if (n<0) { dbg_putc('-'); u=(unsigned)(-n); } else u=(unsigned)n;
    do { b[i++]='0'+(u%10); u/=10; } while(u);
    while(i) dbg_putc(b[--i]);
}

/* =====================================================
 *  Main
 * ===================================================== */
int main(void)
{
    SYSCFG_DL_init();          /* init UART1(user) + UART3(MSPMotor) + SysTick + clock */
    dbg_init();                /* manual init UART0 for debug */
    delay_ms(2000);

    /* enable gyro UART3 RX interrupt */
    NVIC_ClearPendingIRQ(MSPMotor_INST_INT_IRQN);
    NVIC_EnableIRQ(MSPMotor_INST_INT_IRQN);

    /* configure gyro report rate */
    Gyro_ConfigReportRateRx();
    delay_ms(100);

    /* start driver board */
    drv_start();
    delay_ms(200);

    dbg_puts("READY\r\n");

    int loop = 0;
    int16_t angle = 0;
    while (1) {
        /* request gyro data every 100ms */
        delay_ms(100);
        Gyro_RequestData();

        if (gyro_rx_done) {
            gyro_rx_done = 0;
            angle = gyro_angle_raw;  /* store latest angle */
        }

        /* every 500ms: print gyro data + send speeds to driver */
        loop++;
        if (loop >= 5) {
            loop = 0;

            dbg_puts("A="); dbg_putdec((int)angle);
            dbg_puts(" D="); dbg_putdec((int)gyro_dps_raw);
            dbg_puts("\r\n");

            /* use gyro angle to control motors */
            int16_t spd = angle / 10;   /* 0.1 degree units -> rough motor speed */
            if (spd > 50)  spd = 50;
            if (spd < -50) spd = -50;
            drv_set_speeds(spd, -spd, spd/2, -spd/2);
        }
    }
}

/* ---------- UART3 RX ISR (gyro) ---------- */
void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(MSPMotor_INST)) {
        case DL_UART_IIDX_RX:
            Gyro_ParseFrame(DL_UART_Main_receiveData(MSPMotor_INST));
            break;
        default: break;
    }
}

/* ---------- SysTick ---------- */
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while (delay_times != 0);
}

void SysTick_Handler(void)
{
    if (delay_times != 0) delay_times--;
}
