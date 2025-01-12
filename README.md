
# QuadrotorAircraft

## 状态
未完结，更新中

## 项目简介
本项目基于STM32F401RET6开发板实现了一个四轴飞行器，通过MadgWick算法对IMU数据处理后，利用PID进行反馈控制，实现了稳定的姿态控制。
更多说明见博客：https://www.dukiblog.site/?p=47
## 功能特性
- 高效的PID控制：基于串级PID控制和单环PID控制算法，确保飞行器姿态的高稳定性。
- 传感器数据融合：使用MPU6050传感器采集加速度计和陀螺仪数据，并通过Madgwick滤波算法融合这些数据来计算飞行器的姿态（欧拉角）。
- 开源社区支持：提供完整的代码和文档支持，欢迎参与项目的改进和贡献。

## 技术栈
- C，ARM汇编
- 转接板设计与焊接

## 软件架构
1. 传感器读取与数据处理
- 通过I2C接口与MPU6050传感器进行通信，读取加速度计和陀螺仪数据。
- 使用Madgwick滤波算法对传感器数据进行融合，得到飞行器的滚转（Roll）、俯仰（Pitch）、偏航（Yaw）角度。
2. PID控制算法
- 使用串级PID控制器实现对Roll和Pitch角度的控制。
- 对Yaw角度采用单环PID控制进行控制。
- PID控制器的参数可以通过外部配置调整，以应对不同的飞行环境和要求。
3. 电机控制
- 根据PID控制输出计算每个电机的转速，并通过PWM信号控制电调（ESC）以驱动电机。
4. 调试与测试
- 使用串口输出调试信息，包括传感器数据、PID控制输出、飞行器姿态等，方便开发者调试和优化控制算法。

## 安装与配置
### 硬件连接
- STM32开发板：将MPU6050传感器、蓝牙通过转接板连接到STM32开发板。
- 电机与电调（ESC）连接：将电调与STM32的PWM输出引脚连接，电机驱动部分根据PID输出调节转速。
### 软件依赖
- Keil uVision5：用于编译和调试代码。
- STM32CubeMX：用于初始化STM32硬件外设。
### 编译与上传
- 在STM32CubeMX中配置I2C、PWM等外设。
- 使用Keil uVision5进行编译，生成.hex文件。
- 通过ST-Link将固件烧录到STM32开发板。

## 项目结构
```
QuadrotorAircraft/
├── Attitude/                # 姿态控制相关代码
├── BSP/                     # 板级支持包（硬件驱动）
├── Core/                    # 核心代码
├── Drivers/                 # 硬件驱动库
├── MDK-ARM/                 # Keil MDK项目文件
└── QuadrotorAircraft.ioc    # STM32CubeMX配置文件
```
## 维护者
Dukiya(个人博客:https://www.dukiblog.site/)
