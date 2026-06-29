#ifndef LINE_PATROL_H
#define LINE_PATROL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 八路灰度循迹 + 增量式 PID 控制器
 *
 * 传感器: 八路灰度 (黑线=灯灭=高电平)
 * 硬件: OUT1=PA26, OUT2=PA24, OUT3=PB24, OUT4=PA22,
 *       OUT5=PA15, OUT6=PA17, OUT7=PB18, OUT8=PA21
 *       (排针从左到右，直接插杜邦线)
 *
 * 使用:
 *   while(1) {
 *       delay_ms(5);
 *       LinePatrol_Track(10);  // 速度 10 循迹
 *   }
 */

/* PID 参数 — 可在线调整 */
extern float LinePatrol_Kp;
extern float LinePatrol_Ki;
extern float LinePatrol_Kd;

/* 初始化 GPIO (SysConfig 已配置，内部空) */
void LinePatrol_Init(void);

/* 读取 8 路传感器: bit0=OUT1 ... bit7=OUT8 (1=黑线) */
uint8_t LinePatrol_Read(void);

/* 加权计算偏差 (-35~+35, 正=偏右) */
int16_t LinePatrol_CalcDeviation(uint8_t sensors);

/* 增量式 PID */
int16_t LinePatrol_PID(int16_t deviation);

/* 重置 PID 积分 */
void LinePatrol_PID_Reset(void);

/* 一步循迹 (读+PID+驱动) — 最常用 */
void LinePatrol_Track(int16_t base_speed);

/* 调试: 获取上次 PID 输出 */
int16_t LinePatrol_GetLastOutput(void);

#endif
