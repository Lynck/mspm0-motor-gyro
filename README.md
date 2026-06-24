# MSPM0G3507 四轮底盘驱动 + 陀螺仪

## 📋 项目概述

基于 TI MSPM0G3507（天猛星）的四轮麦克纳姆轮底盘控制代码，通过 Modbus RTU 协议分别控制四路电机驱动板和陀螺仪。

## 🔌 硬件连接

| 外设 | 协议 | 引脚 | SysConfig 名称 | 说明 |
|------|------|------|---------------|------|
| 四路电机驱动板 | Modbus RTU (0x0A) | **PA9=TX, PA10=RX** | `user_INST` (UART0) | 115200 8N1 |
| 陀螺仪 | Modbus RTU (0x0A) | **PB2=TX, PB3=RX** | `MSPMotor_INST` (UART3) | 115200 8N1 |
| PC 调试串口 | CH340 (UART0) | **PA9=TX, PA10=RX** | 与电机共用 UART0 | 115200 8N1 |

> ⚠️ **重要**: 电机驱动板和 PC 调试共用 UART0。调试时电机控制会暂停，下载后拔掉调试器即可正常控制电机。

## 🚗 车轮映射

| 寄存器地址 | 电机编号 | 车轮位置 | 方向说明 |
|-----------|---------|---------|---------|
| 0x0000 | 电机0 | **右前轮** | 正数 = 向前 ✅ |
| 0x0001 | 电机1 | **右后轮** | 正数 = 向前 ✅ |
| 0x0002 | 电机2 | **左前轮** | 正数 = 向前 ✅ |
| 0x0003 | 电机3 | **左后轮** | ⚠️ 电机装反，代码自动取反 |

> 左后轮(电机3)电机物理装反，只有负速度才往前转。`Wheels_SetSpeeds()` 函数内部自动处理取反，你调用时**统一用正数表示向前**即可。

## 📁 文件结构（Code/ 文件夹）

```
Code/
├── crc16.h / crc16.c        → Modbus CRC16 查表法校验
├── debug.h / debug.c         → 调试串口 (UART0 printf)
├── motor_ctrl.h / motor_ctrl.c → 四路电机驱动控制 (核心)
└── gyro.h / gyro.c           → 陀螺仪数据读取
```

## 📖 API 参考

### 1. 电机控制 (`motor_ctrl.h`)

```c
// 初始化电机驱动 (SysConfig 自动完成，直接调用即可)
void MotorCtrl_Init(void);

// 启动 / 停止所有电机
void MotorCtrl_Start(void);
void MotorCtrl_Stop(void);

// 设置四轮速度（自动处理左后轮反转）
// fr=右前, rr=右后, fl=左前, rl=左后
// 正数=向前, 负数=向后, 范围建议 [-500, 500]
void Wheels_SetSpeeds(int16_t fr, int16_t rr, int16_t fl, int16_t rl);

// 差分驱动（最常用）
// linear  = 线速度 (正=前进, 负=后退)
// angular = 角速度 (正=顺时针旋转, 负=逆时针旋转)
void Wheels_DiffDrive(int16_t linear, int16_t angular);

// 停止所有轮子
void Wheels_Stop(void);
```

**使用示例:**

```c
// 前进 200
Wheels_DiffDrive(200, 0);

// 后退 150
Wheels_DiffDrive(-150, 0);

// 原地顺时针旋转，速度 100
Wheels_DiffDrive(0, 100);

// 右转（前进 + 顺时针）—— 适合循迹
Wheels_DiffDrive(200, 50);

// 单独控制每个轮子
Wheels_SetSpeeds(100, 100, 100, 100);  // 全速前进
Wheels_SetSpeeds(100, 100, -100, -100); // 原地右转
Wheels_SetSpeeds(0, 0, 0, 0);           // 停止
```

### 2. 陀螺仪 (`gyro.h`)

```c
// 全局变量 (volatile，在中断中更新)
extern volatile int16_t gyro_angle;   // 角度 (单位: 0.01°, 9000 = 90.00°)
extern volatile int16_t gyro_dps;     // 角速度 (单位: 0.01°/s)
extern volatile uint8_t  gyro_rx_done; // 新数据就绪标志

// 初始化陀螺仪 (上电后调用一次)
void Gyro_Init(void);

// 请求数据 (在主循环中每 10ms 调用一次)
void Gyro_RequestData(void);

// 解析响应 (在 UART3 RX 中断中调用)
void Gyro_ParseByte(uint8_t data);
```

**使用示例:**

```c
// ---- 在主循环中 ----
while (1) {
    delay_ms(10);

    // 每 10ms 请求一次陀螺仪数据
    Gyro_RequestData();

    // 检查是否有新数据
    if (gyro_rx_done) {
        gyro_rx_done = 0;  // 清除标志

        int16_t angle = gyro_angle;  // 当前角度 (0.01°)
        int16_t dps   = gyro_dps;    // 当前角速度 (0.01°/s)

        // 例如: 用角度做闭环转向控制
        // 目标角度=0 (保持直线), 当前角度=angle
        // PID 或简单 P 控制: turn = -angle / 10;
        Wheels_DiffDrive(200, -angle / 10);
    }
}
```

### 3. 调试串口 (`debug.h`)

```c
void Debug_Init(void);        // 初始化调试串口
void Debug_Puts(const char*); // 发送字符串
void Debug_PutDec(int val);   // 发送十进制整数
```

**使用示例:**

```c
Debug_Puts("角度=");
Debug_PutDec(gyro_angle);
Debug_Puts("
");
```

### 4. CRC16 (`crc16.h`)

```c
// Modbus CRC16 校验 (内部使用，通常不需要直接调用)
unsigned short CRC16(uint8_t *puchMsg, unsigned short usDataLen);
```

## 🔧 中断配置

陀螺仪使用 UART3 RX 中断接收数据，需在 `empty.c` 中配置:

```c
// 使能 UART3 RX 中断
NVIC_ClearPendingIRQ(MSPMotor_INST_INT_IRQN);
NVIC_EnableIRQ(MSPMotor_INST_INT_IRQN);

// 中断服务函数
void MSPMotor_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(MSPMotor_INST)) {
    case DL_UART_IIDX_RX:
        Gyro_ParseByte(DL_UART_Main_receiveData(MSPMotor_INST));
        break;
    default:
        break;
    }
}
```

## 🚀 快速上手流程

1. **SysConfig 配置**: 确保 `user_INST` (UART0) 和 `MSPMotor_INST` (UART3) 已配置为 115200 8N1, RX 中断使能
2. **接线**: 按硬件连接表接好电机驱动板和陀螺仪
3. **编译**: CCS Theia → Project → Build Project
4. **下载**: 下载到天猛星开发板
5. **测试**: 小车应自动进入角度闭环控制模式

## ⚙️ CCS 工程注意事项

- 本工程使用 **CCS Theia** + **TI Arm Clang 4.0.4**
- MSPM0 SDK 版本: **2.09.00.01**
- `Code/` 文件夹已加入编译路径
- `Driver/` 文件夹 (旧代码) 已加入 `.gitignore`，不再参与编译
- 中文注释使用 **UTF-8 BOM** 编码，CCS 编辑器可正常显示

## 🔄 Git 仓库

[Lynck/mspm0-motor-gyro](https://github.com/Lynck/mspm0-motor-gyro)
