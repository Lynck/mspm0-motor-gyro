#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * ================================================================
 * 电机驱动模块 — 通过 Modbus RTU 控制四路电机驱动板
 *
 * 硬件连接: UART0 (PA9=TX, PA10=RX) 接四路电机驱动板
 *           对应 SysConfig 中 user_INST (UART0)
 *
 * 协议: Modbus RTU, 从机地址 0x0A, 115200 8N1
 *
 * 寄存器映射:
 *   0x0000: 电机0 速度 (int16, R/W) — 右前轮
 *   0x0001: 电机1 速度 (int16, R/W) — 右后轮
 *   0x0002: 电机2 速度 (int16, R/W) — 左前轮
 *   0x0003: 电机3 速度 (int16, R/W) — 左后轮（电机装反，自动取反）
 *   0x0008: 启停控制 (写 0x0001=启动, 写 0x0000=停止)
 * ================================================================
 */

/* ---- 底层电机协议 API ---- */

/* 初始化电机驱动 UART (115200 8N1)，SysConfig 已完成配置 */
void MotorCtrl_Init(void);

/* 启动四路电机（写寄存器 0x0008 = 1） */
void MotorCtrl_Start(void);

/* 停止四路电机（写寄存器 0x0008 = 0） */
void MotorCtrl_Stop(void);

/*
 * 设置四路电机原始速度（不做取反处理）
 * @param motor0 右前轮速度  (int16, 正数向前)
 * @param motor1 右后轮速度  (int16, 正数向前)
 * @param motor2 左前轮速度  (int16, 正数向前)
 * @param motor3 左后轮速度  (int16, 正数向前——调用方已处理反转)
 */
void MotorCtrl_SetRawSpeeds(int16_t motor0, int16_t motor1,
                             int16_t motor2, int16_t motor3);

/* ---- 上层车轮控制 API (推荐使用) ---- */

/*
 * 设置四个轮子速度（自动处理左后轮反转）
 * @param fr 右前轮速度 (正数=向前)
 * @param rr 右后轮速度 (正数=向前)
 * @param fl 左前轮速度 (正数=向前)
 * @param rl 左后轮速度 (正数=向前，内部自动取反)
 *
 * ⚠️ 左后轮电机装反了，只有负速度才往前转
 *    此函数内部已将 rl 自动取反，调用者无需关心
 */
void Wheels_SetSpeeds(int16_t fr, int16_t rr, int16_t fl, int16_t rl);

/*
 * 麦克纳姆轮差分驱动
 * @param linear  线速度分量 (正数=前进, 负数=后退)
 * @param angular 角速度分量 (正数=顺时针, 负数=逆时针)
 *
 * 内部公式（四轮差速）:
 *   右前 = linear - angular
 *   右后 = linear - angular
 *   左前 = linear + angular
 *   左后 = linear + angular
 *
 * 速度范围自动限制在 [-500, 500]
 */
void Wheels_DiffDrive(int16_t linear, int16_t angular);

/* 停止所有轮子 (速度=0) */
void Wheels_Stop(void);

#endif
