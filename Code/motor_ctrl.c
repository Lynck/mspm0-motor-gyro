/*
 * motor_ctrl.c — 四路电机驱动控制 (Modbus RTU 协议)
 *
 * 硬件连接:
 *   MSPM0G3507 UART0 (PA9=TX, PA10=RX) ←→ 四路电机驱动板
 *   协议: Modbus RTU, 从机地址 0x0A, 115200 8N1
 *
 * SysConfig 标识: user_INST (UART0)
 *   PA9=TX  (IOMUX_PINCM21)
 *   PA10=RX (IOMUX_PINCM22)
 *
 * 车轮映射:
 *   电机0 (0x0000) = 右前轮
 *   电机1 (0x0001) = 右后轮
 *   电机2 (0x0002) = 左前轮
 *   电机3 (0x0003) = 左后轮 ← 电机装反，Wheels_SetSpeeds 自动取反
 */

#include "motor_ctrl.h"
#include "crc16.h"

/* 驱动板 UART — 对应 SysConfig Motor_INST (UART1: PA8=TX, PA9=RX) */
#define DRV_UART  Motor_INST

/* Modbus 从机地址 */
#define DRV_ADDR  0x0A

/* ================================================================
 * 内部辅助函数 — 发送字节流
 * ================================================================ */
static void drv_send_bytes(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(DRV_UART, buf[i]);
    }
}

/* ================================================================
 * MotorCtrl_Init — 初始化电机驱动 UART
 * SysConfig 的 SYSCFG_DL_init() 已完成 UART0 配置 (115200 8N1)
 * ================================================================ */
void MotorCtrl_Init(void)
{
    /* UART0 由 SYSCFG_DL_init() 初始化，无需额外配置 */
}

/* ================================================================
 * MotorCtrl_Start — 启动四路电机
 * 写寄存器 0x0008 = 0x0001
 * 协议帧: 0A 06 00 08 00 01 C8 B3
 * ================================================================ */
void MotorCtrl_Start(void)
{
    uint8_t frame[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC8, 0xB3};
    drv_send_bytes(frame, 8);
}

/* ================================================================
 * MotorCtrl_Stop — 停止四路电机
 * 写寄存器 0x0008 = 0x0000
 * 协议帧: 0A 06 00 08 00 00 89 73
 * ================================================================ */
void MotorCtrl_Stop(void)
{
    uint8_t frame[] = {DRV_ADDR, 0x06, 0x00, 0x08, 0x00, 0x00, 0x89, 0x73};
    drv_send_bytes(frame, 8);
}

/* ================================================================
 * MotorCtrl_SetRawSpeeds — 批量写四路电机速度
 *
 * 使用功能码 0x10（写多个寄存器）
 * 起始地址 0x0000, 寄存器数量 4
 *
 * 协议帧 (共15字节):
 *   ADDR 10 0000 0004 08 v0H v0L v1H v1L v2H v2L v3H v3L CRC_L CRC_H
 *
 * ⚠️ 此函数不做任何取反处理，调用者负责方向
 *    上层 Wheels_SetSpeeds() 会自动处理左后轮反转
 * ================================================================ */
void MotorCtrl_SetRawSpeeds(int16_t motor0, int16_t motor1,
                             int16_t motor2, int16_t motor3)
{
    uint8_t frame[15];
    int idx = 0;

    /* 帧头 */
    frame[idx++] = DRV_ADDR;    /* 从机地址 */
    frame[idx++] = 0x10;        /* 功能码: 写多个寄存器 */
    frame[idx++] = 0x00;        /* 起始地址高字节 */
    frame[idx++] = 0x00;        /* 起始地址低字节 (0x0000) */
    frame[idx++] = 0x00;        /* 寄存器数量高字节 */
    frame[idx++] = 0x04;        /* 寄存器数量低字节 (4个) */
    frame[idx++] = 0x08;        /* 数据字节数: 4x2=8 */

    /* 电机0 — 右前轮 (大端序) */
    frame[idx++] = (uint8_t)((motor0 >> 8) & 0xFF);
    frame[idx++] = (uint8_t)(motor0 & 0xFF);

    /* 电机1 — 右后轮 */
    frame[idx++] = (uint8_t)((motor1 >> 8) & 0xFF);
    frame[idx++] = (uint8_t)(motor1 & 0xFF);

    /* 电机2 — 左前轮 */
    frame[idx++] = (uint8_t)((motor2 >> 8) & 0xFF);
    frame[idx++] = (uint8_t)(motor2 & 0xFF);

    /* 电机3 — 左后轮 */
    frame[idx++] = (uint8_t)((motor3 >> 8) & 0xFF);
    frame[idx++] = (uint8_t)(motor3 & 0xFF);

    /* CRC16 追加到帧尾 (低字节在前) */
    uint16_t crc = CRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    drv_send_bytes(frame, idx);
}

/* ================================================================
 * 上层 API — 车轮控制（自动处理左后轮反转）
 *
 * 方向约定:
 *   右前/右后/左前: 正数 = 向前
 *   左后(电机3):    电机装反, 负速度才向前
 *
 * Wheels_SetSpeeds 内部将 rl 自动取反
 * 调用者始终: 正数 = 向前, 负数 = 向后
 * ================================================================ */

/*
 * Wheels_SetSpeeds — 设置四个轮子速度
 * fr=右前, rr=右后, fl=左前, rl=左后
 * 自动处理左后轮反转
 */
void Wheels_SetSpeeds(int16_t fr, int16_t rr, int16_t fl, int16_t rl)
{
    /* 左后轮(电机3)装反了，速度取反后发给驱动板 */
    MotorCtrl_SetRawSpeeds(fr, rr, fl, -rl);
}

/*
 * Wheels_Stop — 停止所有轮子
 */
void Wheels_Stop(void)
{
    MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
}

/*
 * Wheels_DiffDrive — 差分驱动
 *
 * 四轮差速公式:
 *   右前 = linear - angular
 *   右后 = linear - angular
 *   左前 = linear + angular
 *   左后 = linear + angular
 *
 * 速度自动钳制在 [-500, 500] 范围
 */
void Wheels_DiffDrive(int16_t linear, int16_t angular)
{
    int16_t fr = linear - angular;
    int16_t rr = linear - angular;
    int16_t fl = linear + angular;
    int16_t rl = linear + angular;

    /* 限幅 */
    #define SPD_LIMIT 500
    if (fr >  SPD_LIMIT) fr =  SPD_LIMIT;
    if (fr < -SPD_LIMIT) fr = -SPD_LIMIT;
    if (rr >  SPD_LIMIT) rr =  SPD_LIMIT;
    if (rr < -SPD_LIMIT) rr = -SPD_LIMIT;
    if (fl >  SPD_LIMIT) fl =  SPD_LIMIT;
    if (fl < -SPD_LIMIT) fl = -SPD_LIMIT;
    if (rl >  SPD_LIMIT) rl =  SPD_LIMIT;
    if (rl < -SPD_LIMIT) rl = -SPD_LIMIT;

    Wheels_SetSpeeds(fr, rr, fl, rl);
}
