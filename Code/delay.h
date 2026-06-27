#ifndef DELAY_H
#define DELAY_H

#include "ti_msp_dl_config.h"

/* 毫秒延时 (非阻塞) */
void delay_ms(unsigned int ms);

/* 获取系统运行毫秒数 */
unsigned long millis(void);

/* 初始化 SysTick 1ms 定时 */
void Delay_Init(void);

#endif
