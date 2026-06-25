#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 电机驱动模块 — Modbus RTU 控制四路电机驱动板
 *
 * 硬件: UART1 (PA8=TX, PA9=RX), SysConfig 名 Motor_INST
 * 协议: Modbus RTU, 从机地址 0x0A, 115200 8N1
 *
 * 寄存器:
 *   0x0000: 电机0 速度 — 右前轮 (正=向前)
 *   0x0001: 电机1 速度 — 右后轮 (正=向前)
 *   0x0002: 电机2 速度 — 左前轮 (正=向前)
 *   0x0003: 电机3 速度 — 左后轮 (电机装反，Wheels_SetSpeeds 自动取反)
 *   0x0008: 启停控制 (0x0001=启动, 0x0000=停止)
 */

/* 底层 API */
void MotorCtrl_Init(void);
void MotorCtrl_Start(void);
void MotorCtrl_Stop(void);
void MotorCtrl_SetRawSpeeds(int16_t m0, int16_t m1, int16_t m2, int16_t m3);

/* 上层 API (推荐使用) */

/* 设置四轮速度 (正=向前，自动处理左后轮反转) */
void Wheels_SetSpeeds(int16_t fr, int16_t rr, int16_t fl, int16_t rl);

/* 差分驱动: linear=线速度, angular=角速度 (正=顺时针) */
void Wheels_DiffDrive(int16_t linear, int16_t angular);

/* 停止所有轮子 */
void Wheels_Stop(void);

#endif
