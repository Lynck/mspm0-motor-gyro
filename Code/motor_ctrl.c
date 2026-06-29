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
#define WHEEL_LIMIT 20

static int16_t last_fr = 0;
static int16_t last_rr = 0;
static int16_t last_fl = 0;
static int16_t last_rl = 0;

static int16_t clamp_speed(int16_t speed)
{
    if (speed > WHEEL_LIMIT) return WHEEL_LIMIT;
    if (speed < -WHEEL_LIMIT) return -WHEEL_LIMIT;
    return speed;
}

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
    last_fr = fr;
    last_rr = rr;
    last_fl = fl;
    last_rl = rl;
    MotorCtrl_SetRawSpeeds(fr, rr, fl, -rl);
}

void Wheels_GetLastSpeeds(int16_t *fr, int16_t *rr, int16_t *fl, int16_t *rl)
{
    if (fr) *fr = last_fr;
    if (rr) *rr = last_rr;
    if (fl) *fl = last_fl;
    if (rl) *rl = last_rl;
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

    fr = clamp_speed(fr);
    rr = clamp_speed(rr);
    fl = clamp_speed(fl);
    rl = clamp_speed(rl);

    Wheels_SetSpeeds(fr, rr, fl, rl);
}

void Wheels_LineDrive(int16_t base_speed, int16_t steer)
{
    int16_t right = base_speed;
    int16_t left  = base_speed;

    if (steer > 0) {
        right = base_speed - steer;
        left  = base_speed + steer / 2;
    } else if (steer < 0) {
        right = base_speed + (-steer) / 2;
        left  = base_speed + steer;
    }

    if (right < 4) right = 4;
    if (left  < 4) left  = 4;

    right = clamp_speed(right);
    left  = clamp_speed(left);

    Wheels_SetSpeeds(right, right, left, left);
}
