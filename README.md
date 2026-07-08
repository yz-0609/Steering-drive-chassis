# 轮舵哨兵底盘 Keil 工程

这是一个基于 STM32F407IGHx 的 RoboMaster 风格轮舵底盘控制工程，使用 STM32CubeMX 生成外设初始化代码，使用 Keil MDK-ARM 编译，运行 CMSIS-RTOS v1 / FreeRTOS。

## 工程信息

- MCU: STM32F407IGHx
- Keil 工程: `MDK-ARM/standard_robot.uvprojx`
- CubeMX 配置: `standard_robot.ioc`
- 主入口: `Src/main.c`
- RTOS 任务入口: `Src/freertos.c`
- Keil Target: `standard_robot`
- 编译输出目录: `MDK-ARM/standard_tpye_c/`

## 目录结构

```text
application/   机器人业务逻辑：底盘、云台、遥控器、INS、裁判系统、电压检测等
bsp/boards/    板级外设封装：CAN、USART、RC、SPI、I2C、LED、蜂鸣器等
components/    通用组件：PID、AHRS、BMI088、IST8310、OLED、CRC、FIFO 等
Drivers/       STM32 HAL 和 CMSIS
Inc/           CubeMX 生成的头文件
Src/           CubeMX 生成的源文件、main 和 freertos 初始化
Middlewares/   FreeRTOS 和 USB Device 中间件
MDK-ARM/       Keil 工程文件、启动文件和本地编译输出
```

## 当前主要运行链路

`main.c` 完成 HAL、时钟和外设初始化后，初始化 CAN 过滤器、延时、校准参数、遥控器和 USART DMA，然后进入 FreeRTOS 调度。

当前 `Src/freertos.c` 中启用的主要任务包括：

- `calibrate_task`
- `chassis_task`
- `INS_task`
- `led_RGB_flow_task`
- `oled_task`
- `battery_voltage_task`

部分模块代码已保留但任务创建暂时注释，例如 `detect_task`、`gimbal_task`、`referee_usart_task`、`usb_task`、`servo_task`。

底盘控制核心位于 `application/chassis_task.c`，控制周期为 2 ms，流程大致为：

```text
反馈更新 -> 遥控/行为控制量设定 -> 轮舵运动解算 -> PID 控制 -> CAN 发送
```

CAN 电机收发位于 `application/CAN_receive.c`：

- CAN1 接收 3508 底盘驱动电机反馈，常用 ID 为 `0x201` 到 `0x204`
- CAN2 接收 6020 舵向电机反馈，常用 ID 为 `0x205` 到 `0x208`
- `CAN_cmd_chassis()` 发送驱动电机电流
- `CAN_cmd_steering_chassis()` 发送舵向电机电流

## 打开和编译

1. 使用 Keil uVision 打开 `MDK-ARM/standard_robot.uvprojx`。
2. 确认本机已安装 STM32F4xx 相关 Device Pack。
3. 选择 Target `standard_robot`。
4. 直接 Build/Rebuild，编译产物会生成到 `MDK-ARM/standard_tpye_c/`。

## Git 管理说明

仓库保留源码、CubeMX 配置、Keil 工程文件和必要启动/RTE 文件；忽略 Keil 编译输出、下载调试日志、用户界面状态和临时文件。

如果需要保存某次可烧录固件，建议单独建立发布目录或使用 Git tag/Release 管理，不建议把每次构建生成的 `.hex`、`.axf`、`.map` 直接纳入日常版本控制。

