#ifndef DEBUG_H
#define DEBUG_H

#include "ti_msp_dl_config.h"
#include <stdio.h>

/*
 * 调试串口 — UART0 (PA9=TX, PA10=RX), 115200 8N1
 * 通过板载 CH340 连接 PC 实现 printf 输出
 */

void Debug_Init(void);
void Debug_Puts(const char *str);
void Debug_PutDec(int val);

#endif
