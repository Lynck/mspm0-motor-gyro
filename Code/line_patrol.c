/*
 * line_patrol.c — 八路灰度循迹 + 增量式 PID
 *
 * 引脚由 SysConfig GPIO_LinePatrol 模块配置:
 *   LP_OUT1=PA26  LP_OUT2=PA24  LP_OUT3=PA22  LP_OUT4=PA23
 *   LP_OUT5=PA21  LP_OUT6=PA15  LP_OUT7=PA17  LP_OUT8=PB18
 *   黑线=灯灭高电平(1), 白线=灯亮低电平(0)
 */

#include "line_patrol.h"
#include "motor_ctrl.h"
#include "debug.h"

/* PID 参数 — 低速循迹优化 */
float LinePatrol_Kp = 1.0f;
float LinePatrol_Ki = 0.005f;
float LinePatrol_Kd = 2.0f;

static float   pid_integral   = 0.0f;
static int16_t pid_last_dev   = 0;
static int16_t pid_output     = 0;
static int16_t last_valid_dev = 0;

/* OUT1(最左)-4 ... OUT4=-1, OUT5=+1 ... OUT8(最右)+4 */
static const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/* GPIO 由 SysConfig 初始化 */
void LinePatrol_Init(void) {}

/* 读取 8 路 — 使用 SysConfig 生成的宏 */
uint8_t LinePatrol_Read(void)
{
    uint8_t r = 0;

    /* SysConfig 实际分配:
       LP_OUT1=PA26, LP_OUT2=PA24, LP_OUT3=PA22, LP_OUT4=PA23,
       LP_OUT5=PA21, LP_OUT6=PA15, LP_OUT7=PA17, LP_OUT8=PB18 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_26) r |= (1 << 0);  /* OUT1 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_24) r |= (1 << 1);  /* OUT2 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_22) r |= (1 << 2);  /* OUT3 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_23) r |= (1 << 3);  /* OUT4 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_21) r |= (1 << 4);  /* OUT5 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_15) r |= (1 << 5);  /* OUT6 */
    if (GPIOA->DIN31_0 & DL_GPIO_PIN_17) r |= (1 << 6);  /* OUT7 */
    if (GPIOB->DIN31_0 & DL_GPIO_PIN_18) r |= (1 << 7);  /* OUT8 */

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
        return last_valid_dev;

    int16_t d = (int16_t)((sum * 10) / cnt);
    last_valid_dev = d;
    return d;
}

/* 增量式 PID */
int16_t LinePatrol_PID(int16_t dev)
{
    float err = (float)dev;
    float p = LinePatrol_Kp * err;

    if (dev > 5 || dev < -5) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 15.0f)  pid_integral = 15.0f;
        if (pid_integral < -15.0f) pid_integral = -15.0f;
    } else {
        pid_integral = 0.0f;
    }

    float d = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = dev;

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

void LinePatrol_Track(int16_t speed)
{
    uint8_t  s     = LinePatrol_Read();
    int16_t dev    = LinePatrol_CalcDeviation(s);
    int16_t steer  = LinePatrol_PID(dev);
    Wheels_DiffDrive(speed, steer);

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
