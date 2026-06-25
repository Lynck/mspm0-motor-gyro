/*
 * debug.c — UART0 调试串口
 * 通过板载 CH340 连接 PC 实现 printf 调试输出
 */

#include "debug.h"

#define DBG_UART  user_INST

/* 重定向 fputc，让 printf 输出到 UART0 */
int fputc(int ch, FILE *f)
{
    DL_UART_transmitDataBlocking(DBG_UART, (uint8_t)ch);
    return ch;
}

/* 初始化 — SysConfig 已完成 UART0 配置 */
void Debug_Init(void) {}

/* 发送字符串 (不依赖 stdio 缓冲区) */
void Debug_Puts(const char *str)
{
    while (*str) {
        DL_UART_transmitDataBlocking(DBG_UART, (uint8_t)(*str));
        str++;
    }
}

/* 发送十进制整数 */
void Debug_PutDec(int val)
{
    char buf[12];
    int i = 0;
    unsigned int uval;

    if (val == 0) {
        DL_UART_transmitDataBlocking(DBG_UART, '0');
        return;
    }
    if (val < 0) {
        DL_UART_transmitDataBlocking(DBG_UART, '-');
        uval = (unsigned int)(-val);
    } else {
        uval = (unsigned int)val;
    }
    while (uval > 0) {
        buf[i++] = '0' + (uval % 10);
        uval /= 10;
    }
    while (i > 0) {
        i--;
        DL_UART_transmitDataBlocking(DBG_UART, buf[i]);
    }
}
