#ifndef GYRO_H
#define GYRO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * ================================================================
 * 陀螺仪模块 — 通过 Modbus RTU 读取角度和角速度
 *
 * 硬件连接: UART3 (PB2=TX, PB3=RX) 接陀螺仪串口
 *           对应 SysConfig 中 MSPMotor_INST (UART3)
 *
 * 协议: Modbus RTU, 从机地址 0x0A, 115200 8N1
 *
 * 使用流程:
 *   1. 上电后调用 Gyro_Init() 初始化陀螺仪
 *   2. 在主循环中周期性调用 Gyro_RequestData() 请求数据
 *   3. UART3 RX 中断中调用 Gyro_ParseByte() 解析回复
 *   4. 解析完成后 gyro_rx_done = 1，读取 gyro_angle / gyro_dps
 * ================================================================
 */

/* ---- 全局数据 (volatile — 中断中修改) ---- */

/* 陀螺仪角度 (单位: 0.01°, 9000 = 90.00°) */
extern volatile int16_t gyro_angle;

/* 陀螺仪角速度 (单位: 0.01°/s) */
extern volatile int16_t gyro_dps;

/* 数据接收完成标志 (置1表示新数据可用，读取后清0) */
extern volatile uint8_t  gyro_rx_done;

/* ---- API ---- */

/*
 * 初始化陀螺仪，发送校准帧
 * 协议帧: AA 06 01 01 01 AD 00
 */
void Gyro_Init(void);

/*
 * 请求陀螺仪数据（读寄存器 0x0000~0x0001: 角度+角速度）
 * 协议帧: 0A 03 00 00 00 02 C4 B1
 * 通常每 10ms 调用一次
 */
void Gyro_RequestData(void);

/*
 * 解析陀螺仪返回的 Modbus RTU 响应帧
 * 由 UART3 RX 中断服务函数调用
 *
 * 响应帧 (9字节):
 *   0A 03 04 angleH angleL dpsH dpsL CRCL CRCH
 *
 * CRC 校验通过后自动更新 gyro_angle, gyro_dps, gyro_rx_done
 */
void Gyro_ParseByte(uint8_t data);

#endif
