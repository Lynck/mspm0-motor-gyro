/*
 * empty.c — MSPM0G3507 四轮底盘主程序
 */

#include "ti_msp_dl_config.h"
#include "Code/delay.h"
#include "Code/debug.h"
#include "Code/motor_ctrl.h"
#include "Code/line_patrol.h"
#include "Code/gyro.h"

/*
 * 调试模式开关说明（只改 0/1，不要同时打开多个模式）：
 *   MOTOR_SAFE_STOP_MODE  = 1：只反复发送停机命令，用来确认驱动板能不能被主控停住。
 *   MOTOR1_ONLY_TEST_MODE = 1：当前测试模式，只启动驱动板并单独给“电机1/右后轮”写速度 5。
 *   MOTOR_DIAG_MODE       = 1：四个电机按 0->1->2->3 顺序逐个慢速转，方便核对通道映射。
 *
 * 当前为了排查“电机1不转、电机0飞快”，只打开 MOTOR1_ONLY_TEST_MODE。
 */
#define MOTOR_SAFE_STOP_MODE 0
#define MOTOR1_ONLY_TEST_MODE 1
#define MOTOR_DIAG_MODE 0

int main(void)
{
    SYSCFG_DL_init();
    Delay_Init();

    Debug_Init();
    delay_ms(500);
    Debug_Puts("\r\n=== MSPM0G3507 Line Patrol ===\r\n");

#if MOTOR_SAFE_STOP_MODE
    /* 安全停机模式：不调用 MotorCtrl_Start()，理论上所有电机都应该保持不转。 */
    MotorCtrl_EmergencyStop();
    Debug_Puts("[Motor] safe-stop mode: no start command\r\n");
    while (1) {
        /* 反复发停机帧，防止驱动板保留上一次速度状态。 */
        MotorCtrl_EmergencyStop();
        delay_ms(100);
    }
#endif

    /*
     * 正常电机控制前的固定流程：
     *   1. MotorCtrl_Init() 先发停机和四路 0，清理驱动板旧状态；
     *   2. MotorCtrl_Start() 写 0x0008=1，让驱动板进入可运行状态；
     *   3. 后续测试/循迹函数再写具体速度。
     */
    MotorCtrl_Init();
    MotorCtrl_Start();
    delay_ms(100);
    Debug_Puts("[Motor] started\r\n");

#if MOTOR1_ONLY_TEST_MODE
    /*
     * 电机1专测：
     *   MotorCtrl_TestOneMotor(1, 5) 只写驱动板寄存器 0x0001，速度 +5。
     *   如果这时电机0动，说明物理接线/驱动板通道编号与代码编号不一致。
     *   如果仍然所有电机不动，重点检查驱动板“电机2/寄存器0x0001”对应那一路接线。
     */
    Debug_Puts("[Motor] test motor1 only, speed=5\r\n");
    while (1) {
        MotorCtrl_TestOneMotor(1, 5);
        delay_ms(100);
    }
#endif

#if MOTOR_DIAG_MODE
    /* 逐个电机测试：每个电机速度 5，转 2 秒，停 0.5 秒，再测下一路。 */
    Debug_Puts("[Motor] diagnostic: one motor at a time\r\n");
    while (1) {
        MotorCtrl_TestSequence(5, 2000);
    }
#endif

    LinePatrol_Init();
    Debug_Puts("[Sensor] ready\r\n");

    while (1) {
        /* 正常循迹入口：当前被上面的测试模式屏蔽，关闭测试模式后才会运行到这里。 */
        delay_ms(5);
        LinePatrol_Track(10);
    }
}

void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        Gyro_ParseByte(DL_UART_Main_receiveData(MSPMotor_INST));
        break;
    default:
        break;
    }
}
