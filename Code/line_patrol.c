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

/* PID 参数 — 适配 base_speed=10 的低速循迹 */
float LinePatrol_Kp = 0.7f;
float LinePatrol_Ki = 0.0f;
float LinePatrol_Kd = 0.2f;

static float   pid_integral   = 0.0f;
static int16_t pid_last_dev   = 0;
static int16_t pid_output     = 0;
static int16_t last_valid_dev = 0;

/* OUT1(最左)-35 ... OUT4=-5, OUT5=+5 ... OUT8(最右)+35 */
static const int8_t weight[8] = {-35, -25, -15, -5, 5, 15, 25, 35};

/* GPIO 由 SysConfig 初始化 */
void LinePatrol_Init(void) {}

/* 使用 SysConfig 生成的宏读取 8 路传感器 */
uint8_t LinePatrol_Read(void)
{
    uint8_t r = 0;

    if (GPIO_LinePatrol_LP_OUT1_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT1_PIN) r |= (1 << 0);
    if (GPIO_LinePatrol_LP_OUT2_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT2_PIN) r |= (1 << 1);
    if (GPIO_LinePatrol_LP_OUT3_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT3_PIN) r |= (1 << 2);
    if (GPIO_LinePatrol_LP_OUT4_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT4_PIN) r |= (1 << 3);
    if (GPIO_LinePatrol_LP_OUT5_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT5_PIN) r |= (1 << 4);
    if (GPIO_LinePatrol_LP_OUT6_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT6_PIN) r |= (1 << 5);
    if (GPIO_LinePatrol_LP_OUT7_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT7_PIN) r |= (1 << 6);
    if (GPIO_LinePatrol_LP_OUT8_PORT->DIN31_0 & GPIO_LinePatrol_LP_OUT8_PIN) r |= (1 << 7);

    return r;
}

/* 加权偏差 (-35 ~ +35, 正=黑线在右侧) */
int16_t LinePatrol_CalcDeviation(uint8_t sensors)
{
    int16_t sum = 0;
    int cnt = 0;
    for (int i = 0; i < 8; i++)
        if (sensors & (1 << i)) { sum += weight[i]; cnt++; }

    if (cnt == 0 || cnt == 8)
        return last_valid_dev;

    int16_t d = (int16_t)(sum / cnt);
    last_valid_dev = d;
    return d;
}

/* 增量式 PID */
int16_t LinePatrol_PID(int16_t dev)
{
    float err = (float)dev;
    float p = LinePatrol_Kp * err;

    //死区
    if (dev > 1 || dev < -1) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 30.0f)  pid_integral = 30.0f;
        if (pid_integral < -30.0f) pid_integral = -30.0f;
    } else {
        pid_integral = 0.0f;
    }

    float d = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = dev;

    float out = p + pid_integral + d;
    if (out > 10.0f)  out = 10.0f;
    if (out < -10.0f) out = -10.0f;

    pid_output = (int16_t)out;
    if (pid_output > 0 && pid_output < 3) pid_output = 3;
    if (pid_output < 0 && pid_output > -3) pid_output = -3;
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

    static uint8_t motor_div = 0;
    if (++motor_div >= 4) {
        motor_div = 0;
        Wheels_LineDrive(speed, steer);
    }

    static int dbg = 0;
    if (++dbg >= 50) {
        dbg = 0;
        Debug_Puts("OUT=");
        for (int i = 0; i < 8; i++) Debug_PutDec((s >> i) & 1);
        Debug_Puts(" dev=");
        Debug_PutDec(dev);
        Debug_Puts(" steer=");
        Debug_PutDec(steer);
        int16_t fr, rr, fl, rl;
        Wheels_GetLastSpeeds(&fr, &rr, &fl, &rl);
        Debug_Puts(" W=");
        Debug_PutDec(fr);
        Debug_Puts(",");
        Debug_PutDec(rr);
        Debug_Puts(",");
        Debug_PutDec(fl);
        Debug_Puts(",");
        Debug_PutDec(rl);
        Debug_Puts("\r\n");
    }
}

int16_t LinePatrol_GetLastOutput(void)
{
    return pid_output;
}
