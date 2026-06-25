/*
 * line_patrol.c — 八路灰度循迹 + 增量式 PID
 *
 * 引脚 (排针从左到右):
 *   OUT0=PA26  OUT1=PA24  OUT2=PB24  OUT3=PA22
 *   OUT4=PA15  OUT5=PA17  OUT6=PB18  OUT7=PA21
 *   黑线=灯灭高电平(1), 白线=灯亮低电平(0)
 *
 * GPIO 由 SysConfig SYSCFG_DL_init() 自动初始化
 */

#include "line_patrol.h"
#include "motor_ctrl.h"

/* PID 参数 — 可在线调整 */
float LinePatrol_Kp = 3.0f;   /* 比例系数 */
float LinePatrol_Ki = 0.01f;  /* 积分系数 */
float LinePatrol_Kd = 1.5f;   /* 微分系数 */

static float   pid_integral   = 0.0f;
static int16_t pid_last_dev   = 0;
static int16_t pid_output     = 0;
static int16_t last_valid_dev = 0;

/* 8 路权重: 最左 -4 ... 最右 +4 */
static const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/* 引脚定义 */
#define LP0  DL_GPIO_PIN_26  /* PA26, OUT0 最左 */
#define LP1  DL_GPIO_PIN_24  /* PA24, OUT1 */
#define LP2  DL_GPIO_PIN_24  /* PB24, OUT2 */
#define LP3  DL_GPIO_PIN_22  /* PA22, OUT3 */
#define LP4  DL_GPIO_PIN_15  /* PA15, OUT4 */
#define LP5  DL_GPIO_PIN_17  /* PA17, OUT5 */
#define LP6  DL_GPIO_PIN_18  /* PB18, OUT6 */
#define LP7  DL_GPIO_PIN_21  /* PA21, OUT7 最右 */

static const uint32_t lp_pins[8] = {LP0, LP1, LP2, LP3, LP4, LP5, LP6, LP7};
static GPIO_Regs * const lp_ports[8] = {GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOA, GPIOB, GPIOA};

/* GPIO 由 SysConfig 初始化，无需额外操作 */
void LinePatrol_Init(void) {}

/* 逐引脚读取 8 路传感器 */
uint8_t LinePatrol_Read(void)
{
    uint8_t r = 0;
    for (int i = 0; i < 8; i++)
        if (DL_GPIO_readPins(lp_ports[i], lp_pins[i]))
            r |= (1 << i);
    return r;
}

/* 加权计算偏差 (-35 ~ +35, 正=偏右) */
int16_t LinePatrol_CalcDeviation(uint8_t sensors)
{
    int16_t sum = 0;
    int cnt = 0;
    for (int i = 0; i < 8; i++)
        if (sensors & (1 << i)) { sum += weight[i]; cnt++; }
    if (cnt == 0 || cnt == 8) return last_valid_dev;
    int16_t d = (int16_t)((sum * 10) / cnt);
    last_valid_dev = d;
    return d;
}

/* 增量式 PID — 积分分离 + 限幅 */
int16_t LinePatrol_PID(int16_t dev)
{
    float err = (float)dev;

    /* 比例 */
    float p = LinePatrol_Kp * err;

    /* 积分 — |err| >= 20 不累加，防止饱和 */
    if (dev < 20 && dev > -20) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 200.0f)  pid_integral = 200.0f;
        if (pid_integral < -200.0f) pid_integral = -200.0f;
    }

    /* 微分 */
    float d = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = dev;

    /* 合成 + 限幅 */
    float out = p + pid_integral + d;
    if (out > 300.0f)  out = 300.0f;
    if (out < -300.0f) out = -300.0f;

    pid_output = (int16_t)out;
    return pid_output;
}

/* 重置 PID */
void LinePatrol_PID_Reset(void)
{
    pid_integral   = 0.0f;
    pid_last_dev   = 0;
    pid_output     = 0;
    last_valid_dev = 0;
}

/* 一步循迹 — 最常用接口 */
void LinePatrol_Track(int16_t speed)
{
    uint8_t  s     = LinePatrol_Read();
    int16_t dev    = LinePatrol_CalcDeviation(s);
    int16_t steer  = LinePatrol_PID(dev);
    Wheels_DiffDrive(speed, steer);
}

int16_t LinePatrol_GetLastOutput(void)
{
    return pid_output;
}
