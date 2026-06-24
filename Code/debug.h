#ifndef DEBUG_H
#define DEBUG_H

#include "ti_msp_dl_config.h"
#include <stdio.h>

/*
 * 调试串口初始化 — 使用 UART0 (PA9=TX, PA10=RX), 115200 8N1
 * 通过板载 CH340 连接 PC，实现 printf 调试输出
 * SysConfig 中名为 user_INST
 */
void Debug_Init(void);

/*
 * 发送字符串到调试串口 (不依赖 stdio 缓冲区)
 * @param str 以 \0 结尾的字符串
 */
void Debug_Puts(const char *str);

/*
 * 发送十进制整数到调试串口
 * @param val 有符号整数
 */
void Debug_PutDec(int val);

#endif
