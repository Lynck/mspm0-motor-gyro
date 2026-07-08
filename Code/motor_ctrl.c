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
#include "delay.h"

#define DRV_UART  Motor_INST
#define DRV_ADDR  0x0A
#define WHEEL_LIMIT 20
#define MOTOR_WRITE_FRAME_LEN 17

/* 驱动板寄存器地址，来自《四路驱动板.md》的 Modbus RTU 协议表。 */
#define MOTOR_REG_SPEED_BASE 0x0000
#define MOTOR_REG_RUN        0x0008
#define MOTOR_REG_ENC_REV0   0x0009
#define MOTOR_COUNT          4

/*
 * 闭环编码器极性配置：
 *   0 = 不反向编码器反馈；
 *   1 = 反向编码器反馈。
 *
 * 现象判断：
 *   - 当前按官方测试方向发送 (-10, -10, +10, -10) 后，电机2/3正常；
 *   - 电机0飞快、电机1异常，说明右侧两路更像“目标速度方向”和“编码器反馈方向”相反；
 *   - 闭环遇到反馈方向反了，会越纠越大，看起来就是某一路突然飞快或不受控。
 *
 * 因此这里只把电机0/1的编码器反馈取反，电机2/3保持不反向。
 */
#define MOTOR_ENC_REV_M0     1
#define MOTOR_ENC_REV_M1     1
#define MOTOR_ENC_REV_M2     0
#define MOTOR_ENC_REV_M3     0

/* Modbus RTU 帧之间保留一点空闲时间，避免驱动板还没处理完上一帧就收到下一帧。 */
#define MOTOR_FRAME_GAP_MS   5

static int16_t last_fr = 0;
static int16_t last_rr = 0;
static int16_t last_fl = 0;
static int16_t last_rl = 0;

static void drv_send_bytes(const uint8_t *buf, int len);
static void MotorCtrl_ApplyEncoderPolarity(void);

static int16_t clamp_speed(int16_t speed)
{
    if (speed > WHEEL_LIMIT) return WHEEL_LIMIT;
    if (speed < -WHEEL_LIMIT) return -WHEEL_LIMIT;
    return speed;
}

static void drv_write_single(uint16_t reg, uint16_t value)
{
    uint8_t f[8];
    int i = 0;

    /*
     * 功能码 0x06：写单个保持寄存器。
     * 帧格式：地址、功能码、寄存器高低字节、数据高低字节、CRC低字节、CRC高字节。
     */
    f[i++] = DRV_ADDR;
    f[i++] = 0x06;
    f[i++] = (uint8_t)((reg >> 8) & 0xFF);
    f[i++] = (uint8_t)(reg & 0xFF);
    f[i++] = (uint8_t)((value >> 8) & 0xFF);
    f[i++] = (uint8_t)(value & 0xFF);

    uint16_t crc = CRC16(f, i);
    f[i++] = (uint8_t)(crc & 0xFF);
    f[i++] = (uint8_t)((crc >> 8) & 0xFF);

    drv_send_bytes(f, i);
}

/* 发送字节流 */
static void drv_send_bytes(const uint8_t *buf, int len)
{
    uint8_t rx_dump[16];

    /* 发送前清掉驱动板上一次回复残留，避免 RX FIFO 堆积影响后续通信。 */
    DL_UART_Main_drainRXFIFO(DRV_UART, rx_dump, sizeof(rx_dump));

    for (int i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(DRV_UART, buf[i]);
    }

    /* 等最后一个字节真正发完，再留帧间隔。 */
    while (DL_UART_Main_isBusy(DRV_UART)) {
    }

    delay_ms(MOTOR_FRAME_GAP_MS);
}

/* 初始化 — 等待驱动板稳定，并清掉上一次调试可能留下的危险状态 */
void MotorCtrl_Init(void)
{
    /*
     * 上电先停机，再等待驱动板稳定，然后再停一次。
     * 这样做是为了清理驱动板可能保留的旧速度/旧启动状态。
     */
    MotorCtrl_EmergencyStop();
    delay_ms(1200);
    MotorCtrl_EmergencyStop();
    MotorCtrl_ApplyEncoderPolarity();
}

/* 启动电机: 写寄存器 0x0008 = 1 */
void MotorCtrl_Start(void)
{
    /*
     * 启动驱动板后立刻写四路 0，并清一次编码器极性反向标志。
     * 目的是避免闭环驱动板带着旧速度或旧极性配置直接运行。
     */
    drv_write_single(MOTOR_REG_RUN, 0x0001);
    MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
    MotorCtrl_ApplyEncoderPolarity();
    MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
}

/* 停止电机: 写寄存器 0x0008 = 0 */
void MotorCtrl_Stop(void)
{
    drv_write_single(MOTOR_REG_RUN, 0x0000);
}

void MotorCtrl_EmergencyStop(void)
{
    /*
     * 紧急停机组合：
     *   1. 写 0x0008=0，关闭驱动板运行；
     *   2. 再批量写四路速度 0，清掉目标速度寄存器。
     */
    drv_write_single(MOTOR_REG_RUN, 0x0000);
    MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
}

/* 清除四路编码器极性反向标志，避免闭环把某一路越纠越快 */
void MotorCtrl_ClearEncoderReverse(void)
{
    for (uint8_t motor_id = 0; motor_id < MOTOR_COUNT; motor_id++) {
        drv_write_single(MOTOR_REG_ENC_REV0 + motor_id, 0x0000);
    }
}

static void MotorCtrl_ApplyEncoderPolarity(void)
{
    /*
     * 寄存器对应关系来自官方闭环例程：
     *   0x0009 -> 电机0编码器极性
     *   0x000A -> 电机1编码器极性
     *   0x000B -> 电机2编码器极性
     *   0x000C -> 电机3编码器极性
     *
     * 这里不要再统一清零，因为右侧两路现在需要反向反馈。
     */
    drv_write_single(MOTOR_REG_ENC_REV0 + 0, MOTOR_ENC_REV_M0);
    drv_write_single(MOTOR_REG_ENC_REV0 + 1, MOTOR_ENC_REV_M1);
    drv_write_single(MOTOR_REG_ENC_REV0 + 2, MOTOR_ENC_REV_M2);
    drv_write_single(MOTOR_REG_ENC_REV0 + 3, MOTOR_ENC_REV_M3);
}

/* 批量写四路速度 (不做取反，调用方负责方向) */
void MotorCtrl_SetRawSpeeds(int16_t m0, int16_t m1, int16_t m2, int16_t m3)
{
    uint8_t f[MOTOR_WRITE_FRAME_LEN];
    int i = 0;

    /*
     * 功能码 0x10：从 0x0000 开始连续写 4 个速度寄存器。
     * 注意：这里是底层原始速度，不处理左后轮装反；上层 Wheels_SetSpeeds() 才处理方向映射。
     */
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
    /*
     * 车体语义速度：
     *   fr=右前轮，rr=右后轮，fl=左前轮，rl=左后轮。
     * 左后轮电机装反，所以写入驱动板前对 rl 取反。
     */
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

void MotorCtrl_TestOneMotor(uint8_t motor_id, int16_t speed)
{
    int16_t raw_speed;

    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    /*
     * 单电机测试只写一个速度寄存器：
     *   motor_id=0 -> 0x0000，motor_id=1 -> 0x0001，
     *   motor_id=2 -> 0x0002，motor_id=3 -> 0x0003。
     * 电机3是左后轮，机械方向装反，所以测试时也需要取反。
     */
    raw_speed = (motor_id == 3) ? -speed : speed;
    drv_write_single(MOTOR_REG_SPEED_BASE + motor_id, (uint16_t)raw_speed);
}

void MotorCtrl_TestSequence(int16_t speed, unsigned int run_ms)
{
    for (uint8_t motor_id = 0; motor_id < MOTOR_COUNT; motor_id++) {
        MotorCtrl_TestOneMotor(motor_id, speed);
        delay_ms(run_ms);
        MotorCtrl_SetRawSpeeds(0, 0, 0, 0);
        delay_ms(500);
    }
}
