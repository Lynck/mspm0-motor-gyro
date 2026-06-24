/*
 * gyro.c — 陀螺仪数据读取 (Modbus RTU 协议)
 *
 * 硬件连接:
 *   MSPM0G3507 UART3 (PB2=TX, PB3=RX) ←→ 陀螺仪串口
 *   协议: Modbus RTU, 从机地址 0x0A, 115200 8N1
 *
 * SysConfig 标识: MSPMotor_INST (UART3)
 *   PB2=TX (IOMUX_PINCM15)
 *   PB3=RX (IOMUX_PINCM16)
 *
 * 数据格式:
 *   寄存器 0x0000: 角度 (int16, 单位 0.01°)
 *   寄存器 0x0001: 角速度 (int16, 单位 0.01°/s)
 *
 * 使用流程:
 *   1. Gyro_Init()     — 上电初始化
 *   2. Gyro_RequestData() — 每10ms发送查询帧
 *   3. Gyro_ParseByte()   — 在UART3 RX中断中逐字节解析
 *   4. gyro_rx_done==1 时读取 gyro_angle / gyro_dps
 */

#include "gyro.h"
#include "crc16.h"

/* 陀螺仪 UART — 对应 SysConfig MSPMotor_INST (UART3) */
#define GYRO_UART  MSPMotor_INST

/* ---- 全局变量 ---- */
volatile int16_t gyro_angle  = 0;
volatile int16_t gyro_dps    = 0;
volatile uint8_t  gyro_rx_done = 0;

/* ================================================================
 * Gyro_Init — 初始化陀螺仪
 *
 * 发送校准/初始化帧:
 *   AA 06 01 01 01 AD 00
 *   地址=0xAA, 功能码=0x06(写单个寄存器),
 *   寄存器=0x0101, 数据=0x01
 * ================================================================ */
void Gyro_Init(void)
{
    uint8_t frame[] = {0xAA, 0x06, 0x01, 0x01, 0x01, 0xAD, 0x00};

    for (int i = 0; i < 7; i++) {
        DL_UART_transmitDataBlocking(GYRO_UART, frame[i]);
    }
}

/* ================================================================
 * Gyro_RequestData — 请求陀螺仪数据
 *
 * 发送 Modbus 读寄存器帧 (功能码 0x03):
 *   从机地址=0x0A
 *   起始寄存器=0x0000 (角度)
 *   寄存器数量=2 (角度 + 角速度)
 *   完整帧: 0A 03 00 00 00 02 C4 B1
 *
 * 建议每 10ms 调用一次
 * ================================================================ */
void Gyro_RequestData(void)
{
    uint8_t frame[8];
    int idx = 0;

    frame[idx++] = 0x0A;    /* 从机地址 */
    frame[idx++] = 0x03;    /* 功能码: 读多个寄存器 */
    frame[idx++] = 0x00;    /* 起始地址高字节 */
    frame[idx++] = 0x00;    /* 起始地址低字节 (0x0000) */
    frame[idx++] = 0x00;    /* 寄存器数量高字节 */
    frame[idx++] = 0x02;    /* 寄存器数量低字节 (2个) */

    /* CRC16 附加到帧尾 */
    uint16_t crc = CRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    for (int i = 0; i < idx; i++) {
        DL_UART_transmitDataBlocking(GYRO_UART, frame[i]);
    }
}

/* ================================================================
 * Gyro_ParseByte — 状态机解析陀螺仪响应帧
 *
 * 预期响应帧 (9字节):
 *   [0] 0x0A — 从机地址
 *   [1] 0x03 — 功能码
 *   [2] 0x04 — 数据字节数
 *   [3] angleH — 角度高字节
 *   [4] angleL — 角度低字节
 *   [5] dpsH   — 角速度高字节
 *   [6] dpsL   — 角速度低字节
 *   [7] CRCL   — CRC 低字节
 *   [8] CRCH   — CRC 高字节
 *
 * 状态机:
 *   状态0: 等待地址 0x0A
 *   状态1: 等待功能码 0x03
 *   状态2: 等待字节数 0x04
 *   状态3: 接收6字节数据 (角度+角速度+CRC)
 *
 * CRC 校验通过后更新 gyro_angle, gyro_dps, gyro_rx_done
 *
 * ⚠️ 此函数由 UART3 RX 中断调用，需保持执行时间短
 * ================================================================ */
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

    case 1:  /* 等待功能码 0x03 (读寄存器响应) */
        if (data == 0x03) {
            frame[idx++] = data;
            state = 2;
        } else {
            state = 0;  /* 不匹配，重置 */
        }
        break;

    case 2:  /* 等待数据字节数 0x04 */
        if (data == 0x04) {
            frame[idx++] = data;
            state = 3;
        } else {
            state = 0;
        }
        break;

    case 3:  /* 接收数据 + CRC (共6字节) */
        frame[idx++] = data;
        if (idx >= 9) {
            /* 帧收全，验证 CRC */
            uint16_t crc_calc = CRC16(frame, 7);
            uint16_t crc_recv = (uint16_t)frame[7] | ((uint16_t)frame[8] << 8);

            if (crc_calc == crc_recv) {
                /* CRC 正确，解析数据 */
                gyro_angle = (int16_t)(((uint16_t)frame[3] << 8) | frame[4]);
                gyro_dps   = (int16_t)(((uint16_t)frame[5] << 8) | frame[6]);
                gyro_rx_done = 1;  /* 通知主循环: 有新数据 */
            }
            state = 0;  /* 回到初始状态，等待下一帧 */
        }
        break;

    default:
        state = 0;
        break;
    }
}
