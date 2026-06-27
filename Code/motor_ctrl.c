/*
 * motor_ctrl.c — 四路电机驱动 (Modbus RTU)
 *
 * 硬件: UART1 (PA8=TX, PA9=RX) — SysConfig Motor_INST
 * 协议: Modbus RTU, 地址 0x0A, 115200 8N1
 *
 * 车轮映射:
 *   电机0 = 右前轮 (正=向前)
 *   电机1 = 右后轮 (正=向前)
 *   电机2 = 左前轮 (正=向前)
 *   电机3 = 左后轮 (电机装反，Wheels_SetSpeeds 自动取反)
 */

#include "motor_ctrl.h"
#include "crc16.h"

#define DRV_UART  Motor_INST
#define DRV_ADDR  0x0A

/* 发送字节流 */
static void drv_send_bytes(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        DL_UART_transmitDataBlocking(DRV_UART, buf[i]);
}

/* 初始化 — SysConfig 已完成 UART1 配置 */
void MotorCtrl_Init(void) {}

/* 启动电机: 写寄存器 0x0008 = 1 */
void MotorCtrl_Start(void)
{
    uint8_t f[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC8, 0xB3};
    drv_send_bytes(f, 8);
}

/* 停止电机: 写寄存器 0x0008 = 0 */
void MotorCtrl_Stop(void)
{
    uint8_t f[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x00, 0x89, 0x73};
    drv_send_bytes(f, 8);
}

/* 批量写四路速度 (不做取反，调用方负责方向) */
void MotorCtrl_SetRawSpeeds(int16_t m0, int16_t m1, int16_t m2, int16_t m3)
{
    uint8_t f[15];
    int i = 0;

    f[i++] = DRV_ADDR;  f[i++] = 0x10;  /* 写多个寄存器 */
    f[i++] = 0x00;      f[i++] = 0x00;  /* 起始地址 0x0000 */
    f[i++] = 0x00;      f[i++] = 0x04;  /* 4 个寄存器 */
    f[i++] = 0x08;                       /* 8 字节数据 */

    f[i++] = (uint8_t)((m0 >> 8) & 0xFF);
    f[i++] = (uint8_t)(m0 & 0xFF);
    f[i++] = (uint8_t)((m1 >> 8) & 0xFF);
    f[i++] = (uint8_t)(m1 & 0xFF);
    f[i++] = (uint8_t)((m2 >> 8) & 0xFF);
    f[i++] = (uint8_t)(m2 & 0xFF);
    f[i++] = (uint8_t)((m3 >> 8) & 0xFF);
    f[i++] = (uint8_t)(m3 & 0xFF);

    uint16_t crc = CRC16(f, i);
    f[i++] = (uint8_t)(crc & 0xFF);
    f[i++] = (uint8_t)((crc >> 8) & 0xFF);

    drv_send_bytes(f, i);
}

/* ================================================================
 * 上层 API — 自动处理左后轮反转
 * ================================================================ */

/* 设置四轮速度 (左后轮自动取反) */
void Wheels_SetSpeeds(int16_t fr, int16_t rr, int16_t fl, int16_t rl)
{
    MotorCtrl_SetRawSpeeds(fr, rr, fl, -rl);
}

/* 停止 */
void Wheels_Stop(void)
{
    MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
}

/* 差分驱动 */
void Wheels_DiffDrive(int16_t linear, int16_t angular)
{
    int16_t fr = linear - angular;
    int16_t rr = linear - angular;
    int16_t fl = linear + angular;
    int16_t rl = linear + angular;

    #define LIMIT 20
    if (fr >  LIMIT) fr =  LIMIT;  if (fr < -LIMIT) fr = -LIMIT;
    if (rr >  LIMIT) rr =  LIMIT;  if (rr < -LIMIT) rr = -LIMIT;
    if (fl >  LIMIT) fl =  LIMIT;  if (fl < -LIMIT) fl = -LIMIT;
    if (rl >  LIMIT) rl =  LIMIT;  if (rl < -LIMIT) rl = -LIMIT;

    Wheels_SetSpeeds(fr, rr, fl, rl);
}
