#ifndef GYRO_H
#define GYRO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 陀螺仪模块 — UART3 (PB2=TX, PB3=RX) Modbus RTU
 *
 * 协议: Modbus RTU, 地址 0x0A, 115200 8N1
 * 寄存器: 0x0000=角度(0.01°), 0x0001=角速度(0.01°/s)
 *
 * 使用流程:
 *   1. Gyro_Init()         — 上电初始化
 *   2. Gyro_RequestData()  — 每 10ms 请求数据
 *   3. Gyro_ParseByte()    — UART3 RX 中断中逐字节解析
 *   4. gyro_rx_done==1 时读取 gyro_angle / gyro_dps
 */

extern volatile int16_t gyro_angle;   /* 角度 (单位: 0.01°) */
extern volatile int16_t gyro_dps;     /* 角速度 (单位: 0.01°/s) */
extern volatile uint8_t  gyro_rx_done; /* 数据就绪标志 */

void Gyro_Init(void);
void Gyro_RequestData(void);
void Gyro_ParseByte(uint8_t data);

#endif
