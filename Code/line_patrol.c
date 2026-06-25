/*
 * line_patrol.c ? ?????? + ??? PID ???
 *
 * ??: ??????? OUT1~OUT8
 *       ??? PB4~PB11
 *       PINCM17 ? PB4(OUT1) ... PINCM24 ? PB11(OUT8)
 *       ??=?????(1), ??=?????(0)
 *
 * ??:
 *   motor_ctrl.h ? Wheels_DiffDrive()
 */

#include "line_patrol.h"
#include "motor_ctrl.h"

/* ---- PID ?? (?????????) ---- */
float LinePatrol_Kp = 3.0f;   /* ???? */
float LinePatrol_Ki = 0.01f;  /* ???? */
float LinePatrol_Kd = 1.5f;   /* ???? */

/* ---- PID ???? ---- */
static float pid_integral  = 0.0f;  /* ???? */
static int16_t pid_last_dev = 0;    /* ????? */
static int16_t pid_output   = 0;    /* ???? */
static int16_t last_valid_dev = 0;  /* ??????? */

/*
 * ??????? ? 8 ?????
 * OUT1(??): -4, OUT2: -3, OUT3: -2, OUT4: -1
 * OUT5: +1, OUT6: +2, OUT7: +3, OUT8(??): +4
 */
static const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/* ---- PB4~PB11 ???? ---- */
#define LP_PINS  (DL_GPIO_PIN_4 | DL_GPIO_PIN_5 | DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | \
                  DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11)

/* ================================================================
 * LinePatrol_Init ? ??? 8 ? GPIO ????? + ??
 *
 * ?? DL_GPIO_initDigitalInputFeatures API
 * ?????? PINCM ????
 * ================================================================ */
void LinePatrol_Init(void)
{
    /* PB4~PB11: PINCM17~24, ????, ???, ???, ??? */
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM17,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM18,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM19,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM20,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM21,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM22,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM23,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM24,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* ================================================================
 * LinePatrol_Read ? ????? 8 ????
 *
 * @return uint8_t ??:
 *         bit0=PB4(OUT1) ... bit7=PB11(OUT8)
 *         1=??(???), 0=??(???)
 * ================================================================ */
uint8_t LinePatrol_Read(void)
{
    uint32_t raw = DL_GPIO_readPins(GPIOB, LP_PINS);
    return (uint8_t)((raw >> 4) & 0xFF);
}

/* ================================================================
 * LinePatrol_CalcDeviation ? ??????
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

    /* ?????: ????? */
    if (active_count == 0 || active_count == 8) {
        return last_valid_dev;
    }

    int16_t dev = (int16_t)((weighted_sum * 10) / active_count);
    last_valid_dev = dev;
    return dev;
}

/* ================================================================
 * LinePatrol_PID ? ??? PID
 *
 * ????: |err| >= 20 ??????
 * ????: [-200, 200]
 * ????: [-300, 300]
 * ================================================================ */
int16_t LinePatrol_PID(int16_t deviation)
{
    float err  = (float)deviation;

    /* P */
    float p_term = LinePatrol_Kp * err;

    /* I ? ???? */
    if (deviation < 20 && deviation > -20) {
        pid_integral += LinePatrol_Ki * err;
        if (pid_integral > 200.0f)  pid_integral = 200.0f;
        if (pid_integral < -200.0f) pid_integral = -200.0f;
    }

    /* D */
    float d_term = LinePatrol_Kd * (err - (float)pid_last_dev);
    pid_last_dev = deviation;

    /* ?? + ?? */
    float output_f = p_term + pid_integral + d_term;
    if (output_f > 300.0f)  output_f = 300.0f;
    if (output_f < -300.0f) output_f = -300.0f;

    pid_output = (int16_t)output_f;
    return pid_output;
}

/* ================================================================
 * LinePatrol_PID_Reset ? ?? PID
 * ================================================================ */
void LinePatrol_PID_Reset(void)
{
    pid_integral   = 0.0f;
    pid_last_dev   = 0;
    pid_output     = 0;
    last_valid_dev = 0;
}

/* ================================================================
 * LinePatrol_Track ? ????
 * ================================================================ */
void LinePatrol_Track(int16_t base_speed)
{
    uint8_t  sensors   = LinePatrol_Read();
    int16_t deviation = LinePatrol_CalcDeviation(sensors);
    int16_t steer     = LinePatrol_PID(deviation);

    Wheels_DiffDrive(base_speed, steer);
}

/* ================================================================
 * LinePatrol_GetLastOutput ? ???
 * ================================================================ */
int16_t LinePatrol_GetLastOutput(void)
{
    return pid_output;
}
