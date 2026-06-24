/*
 * debug.c — UART0 调试串口 (PA9=TX, PA10=RX)
 * 对应 SysConfig 中 user_INST (UART0, 115200 8N1)
 */

#include "debug.h"

/* 调试 UART — 对应 SysConfig user_INST */
#define DBG_UART  user_INST

/* ---------------------------------------------------------------
 * fputc 重定向 — 让 printf 输出到 DBG_UART
 * CCS 标准库会自动调用此函数
 * --------------------------------------------------------------- */
int fputc(int ch, FILE *f)
{
    DL_UART_transmitDataBlocking(DBG_UART, (uint8_t)ch);
    return ch;
}

/* ---------------------------------------------------------------
 * Debug_Init — 初始化调试串口
 * SysConfig 的 SYSCFG_DL_init() 已完成 UART0 时钟和引脚配置
 * 这里只需确认波特率等参数正确即可
 * --------------------------------------------------------------- */
void Debug_Init(void)
{
    /* UART0 由 SYSCFG_DL_init() 初始化，无需额外配置 */
}

/* ---------------------------------------------------------------
 * Debug_Puts — 发送字符串（不依赖 printf 的缓冲区）
 * --------------------------------------------------------------- */
void Debug_Puts(const char *str)
{
    while (*str) {
        DL_UART_transmitDataBlocking(DBG_UART, (uint8_t)(*str));
        str++;
    }
}

/* ---------------------------------------------------------------
 * Debug_PutDec — 发送十进制整数
 * 用于快速输出数值型调试信息，避免 printf 带来的代码膨胀
 * --------------------------------------------------------------- */
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
