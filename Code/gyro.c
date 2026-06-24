/*
 * gyro.c ? ????????? (Modbus RTU ??)
 *
 * ????:
 *   MSPM0G3507 UART3 (PB2=TX, PB3=RX) ? ?????
 *   ??: Modbus RTU, ???? 0x0A, 115200 8N1
 *
 * ??????:
 *   1. ??? Gyro_Init() ?? AA 06 01 01 01 AD 00 ??????
 *   2. ???? 10ms ?? Gyro_RequestData() ?????
 *   3. UART3 RX ??????? Gyro_ParseByte()
 *   4. ?????? gyro_rx_done = 1
 *
 * ??: ???????? UART3 ? MSPMotor_INST (SysConfig ???)
 *       PB2=TX, PB3=RX, ?? RX ??
 */

#include "gyro.h"
#include "crc16.h"

/* ??? UART ? ?? SysConfig ???? MSPMotor_INST (UART3) */
#define GYRO_UART  MSPMotor_INST

/* ???? */
volatile int16_t gyro_angle = 0;
volatile int16_t gyro_dps   = 0;
volatile uint8_t  gyro_rx_done = 0;

/* ================================================================
 * Gyro_Init ? ???????????
 *
 * ???: AA 06 01 01 01 AD 00
 *   ??=0xAA, ???=0x06, ???=0x0101, ??=0x01
 *   CRC ??? 0x00AD (?????????)
 * ================================================================ */
void Gyro_Init(void)
{
    /* ?????? (?????????????) */
    uint8_t frame[] = {0xAA, 0x06, 0x01, 0x01, 0x01, 0xAD, 0x00};

    for (int i = 0; i < 7; i++) {
        DL_UART_transmitDataBlocking(GYRO_UART, frame[i]);
    }
}

/* ================================================================
 * Gyro_RequestData ? ????????
 *
 * ???: 0A 03 00 00 00 02 C4 B1
 *   ??=0x0A, ???=0x03 (??????)
 *   ????=0x0000, ?????=2 (??+???)
 *   CRC16 ??
 * ================================================================ */
void Gyro_RequestData(void)
{
    uint8_t frame[8];
    int idx = 0;

    frame[idx++] = 0x0A;    /* ???? */
    frame[idx++] = 0x03;    /* ???: ?????? */
    frame[idx++] = 0x00;    /* ??????? */
    frame[idx++] = 0x00;    /* ??????? */
    frame[idx++] = 0x00;    /* ???????? */
    frame[idx++] = 0x02;    /* ????????: 2 ? */

    /* CRC16 ?? */
    uint16_t crc = CRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFF);        /* CRC ??? */
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);  /* CRC ??? */

    for (int i = 0; i < idx; i++) {
        DL_UART_transmitDataBlocking(GYRO_UART, frame[i]);
    }
}

/* ================================================================
 * Gyro_ParseByte ? ???????? Modbus RTU ???
 *
 * ????? (9 ??):
 *   0A 03 04 angleH angleL dpsH dpsL CRCL CRCH
 *
 * ????:
 *   state 0: ??? 0x0A
 *   state 1: ???? 0x03
 *   state 2: ???? 0x04
 *   state 3: ? 6 ???? (2??? + 2???? + 2?CRC)
 *   CRC ?????? gyro_rx_done = 1
 *
 * ?? MSPMotor_INST_IRQHandler ???
 * ================================================================ */
void Gyro_ParseByte(uint8_t data)
{
    static uint8_t state = 0;
    static uint8_t frame[9];
    static uint8_t idx  = 0;

    switch (state) {
    case 0:  /* ???? 0x0A */
        if (data == 0x0A) {
            idx = 0;
            frame[idx++] = data;
            state = 1;
        }
        break;

    case 1:  /* ????? 0x03 */
        if (data == 0x03) {
            frame[idx++] = data;
            state = 2;
        } else {
            state = 0;  /* ?????? */
        }
        break;

    case 2:  /* ????? 0x04 */
        if (data == 0x04) {
            frame[idx++] = data;
            state = 3;
        } else {
            state = 0;
        }
        break;

    case 3:  /* ??? 6 ??: ??H, ??L, ???H, ???L, CRCL, CRCH */
        frame[idx++] = data;
        if (idx >= 9) {
            /* CRC ??: ? 7 ?? (??+???+???+??+???) */
            uint16_t crc_calc = CRC16(frame, 7);
            uint16_t crc_recv = (uint16_t)frame[7] | ((uint16_t)frame[8] << 8);

            if (crc_calc == crc_recv) {
                /* ????: ?? (??? 0x0000) */
                gyro_angle = (int16_t)(((uint16_t)frame[3] << 8) | frame[4]);

                /* ????: ??? (??? 0x0001) */
                gyro_dps   = (int16_t)(((uint16_t)frame[5] << 8) | frame[6]);

                gyro_rx_done = 1;  /* ???????? */
            }
            state = 0;  /* ????? */
        }
        break;

    default:
        state = 0;
        break;
    }
}
