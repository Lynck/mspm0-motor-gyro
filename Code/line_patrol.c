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

/* PID 参数 — 低速循迹优化 */
float LinePatrol_Kp = 1.0f;   /* 比例 */
float LinePatrol_Ki = 0.005f; /* 积分 (极小，抑制震荡) */
float LinePatrol_Kd = 2.0f;   /* 微分 (增大，快速抑制震荡) */

static float   pid_integral   = 0.0f;
static int16_t pid_last_dev   = 0;
static int16_t pid_output     = 0;
static int16_t last_valid_dev = 0;

/* 8 路权重: OUT1(最左) -4 ... OUT4=-1, OUT5=+1 ... OUT8(最右) +4 */
static const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/* GPIO 由 SysConfig 初始化 */
void LinePatrol_Init(void) {}

/* 引脚定义 */
#define LP0_P  GPIOA
#define LP0_M  DL_GPIO_PIN_26
#define LP1_P  GPIOA
#define LP1_M  DL_GPIO_PIN_24
#define LP2_P  GPIOB
#define LP2_M  DL_GPIO_PIN_24
#define LP3_P  GPIOA
#define LP3_M  DL_GPIO_PIN_22
#define LP4_P  GPIOA
#define LP4_M  DL_GPIO_PIN_15
#define LP5_P  GPIOA
#define LP5_M  DL_GPIO_PIN_17
#define LP6_P  GPIOB
#define LP6_M  DL_GPIO_PIN_18
#define LP7_P  GPIOA
#define LP7_M  DL_GPIO_PIN_21

/* GPIO 由 SysConfig 初始化 */

/* 逐引脚读取 — 每次调用都重新读硬件 */
uint8_t LinePatrol_Read(void)
{
    uint8_t r = 0;
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_26) r |= (1 << 0);
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_24) r |= (1 << 1);
    if (GPIOB->DIN31_0 & DL_GPIO_PIN_24) r |= (1 << 2);
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_22) r |= (1 << 3);
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_15) r |= (1 << 4);
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_17) r |= (1 << 5);
    if (GPIOB->DIN31_0 & DL_GPIO_PIN_18) r |= (1 << 6);
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_21) r |= (1 << 7);
    return r;
}

/* 加权偏差 (-35~+35, 正=偏右) */
int16_t LinePatrol_CalcDeviation(uint8_t sensors)
{
    int16_t sum = 0;
    int cnt = 0;
    for (int i = 0; i < 8; i++)
        if (sensors & (1 << i)) { sum += weight[i]; cnt++; }

    if (cnt == 0 || cnt == 8)
        return last_valid_dev;  /* 全白/全黑: 保持上次偏差 */

    /* 归一化 */
    int16_t d = (int16_t)((sum * 10) / cnt);
    last_valid_dev = d;
    return d;
}

/* 增量式 PID */
int16_t LinePatrol_PID(int16_t dev)
{
    float err = (float)dev;

    /* P */
    float p = LinePatrol_Kp * err;

    /* I — 偏差很小时清零积分(防止左右摆)，偏差>5才累加 */
    if (dev > 5 || dev < -5) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 15.0f)  pid_integral = 15.0f;
        if (pid_integral < -15.0f) pid_integral = -15.0f;
    } else {
        pid_integral = 0.0f;  /* 偏差小时清零积分 */
    }

    /* D */
    float d = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = dev;

    /* 合成 + 限幅 */
    float out = p + pid_integral + d;
    if (out > 20.0f)  out = 20.0f;
    if (out < -20.0f) out = -20.0f;

    pid_output = (int16_t)out;
    return pid_output;
}

void LinePatrol_PID_Reset(void)
{
    pid_integral   = 0.0f;
    pid_last_dev   = 0;
    pid_output     = 0;
    last_valid_dev = 0;
}

/* 一步循迹 */
#include "debug.h"

void LinePatrol_Track(int16_t speed)
{
    uint8_t  s     = LinePatrol_Read();
    int16_t dev    = LinePatrol_CalcDeviation(s);
    int16_t steer  = LinePatrol_PID(dev);
    Wheels_DiffDrive(speed, steer);

    /* 每 50 次输出传感器状态 (调试用) */
    static int dbg = 0;
    if (++dbg >= 50) {
        dbg = 0;
        Debug_Puts("S=");
        for (int i = 7; i >= 0; i--) Debug_PutDec((s >> i) & 1);
        Debug_Puts(" dev=");
        Debug_PutDec(dev);
        Debug_Puts(" steer=");
        Debug_PutDec(steer);
        Debug_Puts("\r\n");
    }
}

int16_t LinePatrol_GetLastOutput(void)
{
    return pid_output;
}
