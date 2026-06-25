/*
 * line_patrol.c ? ?????? + ??? PID ???
 *
 * ??: ??????? OUT1~OUT8
 *       ??? SysConfig GPIO_LinePatrol ????:
 *       OUT0=PA26 OUT1=PA24 OUT2=PB24 OUT3=PA22
 *       OUT4=PA15 OUT5=PA17 OUT6=PB18 OUT7=PA21
 *       ??=?????(1), ??=?????(0)
 *
 * GPIO ???? SYSCFG_DL_init() ???? (SysConfig ??)
 * ??: motor_ctrl.h ? Wheels_DiffDrive()
 */

#include "line_patrol.h"
#include "motor_ctrl.h"

/* ---- PID ?? (?????????) ---- */
float LinePatrol_Kp = 3.0f;   /* ???? */
float LinePatrol_Ki = 0.01f;  /* ???? */
float LinePatrol_Kd = 1.5f;   /* ???? */

/* ---- PID ???? ---- */
static float pid_integral  = 0.0f;
static int16_t pid_last_dev = 0;
static int16_t pid_output   = 0;
static int16_t last_valid_dev = 0;

/* ?????: OUT1(??) -4 ... OUT8(??) +4 */
static const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/* ???? ? ?? SysConfig ???? */
#define LP_PIN0  DL_GPIO_PIN_26   /* PA26 = OUT1 */
#define LP_PIN1  DL_GPIO_PIN_24   /* PA24 = OUT2 */
#define LP_PIN2  DL_GPIO_PIN_22   /* PA22 = OUT3 */
#define LP_PIN3  DL_GPIO_PIN_23   /* PA23 = OUT4 */
#define LP_PIN4  DL_GPIO_PIN_21   /* PA21 = OUT5 */
#define LP_PIN5  DL_GPIO_PIN_15   /* PA15 = OUT6 */
#define LP_PIN6  DL_GPIO_PIN_17   /* PA17 = OUT7 */
#define LP_PIN7  DL_GPIO_PIN_18   /* PB18 = OUT8 */

static const uint32_t lp_pins[8] = {LP_PIN0, LP_PIN1, LP_PIN2, LP_PIN3,
                                     LP_PIN4, LP_PIN5, LP_PIN6, LP_PIN7};
static const GPIO_Regs * const lp_ports[8] = {GPIOA, GPIOA, GPIOA, GPIOA,
                                                   GPIOA, GPIOA, GPIOA, GPIOB};

/* ================================================================
 * LinePatrol_Init
 * SysConfig SYSCFG_DL_init() ???? PINCM + ??
 * ================================================================ */
void LinePatrol_Init(void)
{
    /* GPIO ?? SYSCFG_DL_init() ??? */
}

/* ================================================================
 * LinePatrol_Read ? ????? 8 ?
 * @return uint8_t bit0=OUT1 ... bit7=OUT8 (1=??)
 * ================================================================ */
uint8_t LinePatrol_Read(void)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        if (DL_GPIO_readPins((GPIO_Regs *)lp_ports[i], lp_pins[i])) {
            result |= (1 << i);
        }
    }
    return result;
}

/* ================================================================
 * LinePatrol_CalcDeviation ? ????
 * ================================================================ */
int16_t LinePatrol_CalcDeviation(uint8_t sensors)
{
    int16_t weighted_sum = 0;
    int     active_count = 0;

    for (int i = 0; i < 8; i++) {
        if (sensors & (1 << i)) {
            weighted_sum += weight[i];
            active_count++;
        }
    }

    if (active_count == 0 || active_count == 8) {
        return last_valid_dev;
    }

    int16_t dev = (int16_t)((weighted_sum * 10) / active_count);
    last_valid_dev = dev;
    return dev;
}

/* ================================================================
 * LinePatrol_PID ? ??? PID (???? + ??)
 * ================================================================ */
int16_t LinePatrol_PID(int16_t deviation)
{
    float err = (float)deviation;

    /* P */
    float p_term = LinePatrol_Kp * err;

    /* I ? |err| >= 20 ??? */
    if (deviation < 20 && deviation > -20) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 200.0f)  pid_integral = 200.0f;
        if (pid_integral < -200.0f) pid_integral = -200.0f;
    }

    /* D */
    float d_term = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = deviation;

    float output_f = p_term + pid_integral + d_term;
    if (output_f > 300.0f)  output_f = 300.0f;
    if (output_f < -300.0f) output_f = -300.0f;

    pid_output = (int16_t)output_f;
    return pid_output;
}

void LinePatrol_PID_Reset(void)
{
    pid_integral = 0.0f;
    pid_last_dev = 0;
    pid_output   = 0;
    last_valid_dev = 0;
}

void LinePatrol_Track(int16_t base_speed)
{
    uint8_t  sensors   = LinePatrol_Read();
    int16_t deviation = LinePatrol_CalcDeviation(sensors);
    int16_t steer     = LinePatrol_PID(deviation);
    Wheels_DiffDrive(base_speed, steer);
}

int16_t LinePatrol_GetLastOutput(void)
{
    return pid_output;
}
