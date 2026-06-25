/*
 * gyro.c — 陀螺仪数据读取 (Modbus RTU)
 *
 * 硬件: UART3 (PB2=TX, PB3=RX) — SysConfig MSPMotor_INST
 * 协议: Modbus RTU, 地址 0x0A, 115200 8N1
 *
 * 响应帧 (9字节): 0A 03 04 angleH angleL dpsH dpsL CRCL CRCH
 */

#include "gyro.h"
#include "crc16.h"

#define GYRO_UART  MSPMotor_INST

volatile int16_t gyro_angle  = 0;
volatile int16_t gyro_dps    = 0;
volatile uint8_t  gyro_rx_done = 0;

/* 初始化陀螺仪 — 发送校准帧 AA 06 01 01 01 AD 00 */
void Gyro_Init(void)
{
    uint8_t f[] = {0xAA, 0x06, 0x01, 0x01, 0x01, 0xAD, 0x00};
    for (int i = 0; i < 7; i++)
        DL_UART_transmitDataBlocking(GYRO_UART, f[i]);
}

/* 请求数据 — 读寄存器 0x0000(角度) + 0x0001(角速度) */
void Gyro_RequestData(void)
{
    uint8_t f[8];
    int i = 0;
    f[i++] = 0x0A;  f[i++] = 0x03;
    f[i++] = 0x00;  f[i++] = 0x00;
    f[i++] = 0x00;  f[i++] = 0x02;

    uint16_t crc = CRC16(f, i);
    f[i++] = (uint8_t)(crc & 0xFF);
    f[i++] = (uint8_t)((crc >> 8) & 0xFF);

    for (int j = 0; j < i; j++)
        DL_UART_transmitDataBlocking(GYRO_UART, f[j]);
}

/* 状态机解析响应帧 — UART3 RX 中断中调用 */
void Gyro_ParseByte(uint8_t data)
{
    static uint8_t state = 0;
    static uint8_t frame[9];
    static uint8_t idx  = 0;

    switch (state) {
    case 0:  /* 等待从机地址 0x0A */
        if (data == 0x0A) {
            idx = 0;
            frame[idx++] = data;
            state = 1;
        }
        break;
    case 1:  /* 等待功能码 0x03 */
        if (data == 0x03) {
            frame[idx++] = data;
            state = 2;
        } else {
            state = 0;
        }
        break;
    case 2:  /* 等待字节数 0x04 */
        if (data == 0x04) {
            frame[idx++] = data;
            state = 3;
        } else {
            state = 0;
        }
        break;
    case 3:  /* 接收 6 字节数据: angleH, angleL, dpsH, dpsL, CRCL, CRCH */
        frame[idx++] = data;
        if (idx >= 9) {
            uint16_t crc_calc = CRC16(frame, 7);
            uint16_t crc_recv = (uint16_t)frame[7] | ((uint16_t)frame[8] << 8);
            if (crc_calc == crc_recv) {
                gyro_angle = (int16_t)(((uint16_t)frame[3] << 8) | frame[4]);
                gyro_dps   = (int16_t)(((uint16_t)frame[5] << 8) | frame[6]);
                gyro_rx_done = 1;  /* 通知主循环: 新数据就绪 */
            }
            state = 0;
        }
        break;
    default:
        state = 0;
        break;
    }
}
