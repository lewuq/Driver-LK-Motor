# Driver-LK-Motor

**🌐 [English](README.md) | [中文](README_CN.md)**

Driver-LK-Motor 提供 LK Motor MS3506V3 和 MS3008V3 15-bit 经典 CAN 版本的跨平台纯 C 驱动，以及 PlatformIO/Zephyr 参考工程。独立驱动不绑定 MCU、CAN 外设、RTOS 或 UI，上层只需通过一个发送回调完成适配。

## 版本迭代

| 版本 | 更新日期 | 主要内容 |
| --- | --- | --- |
| V0.2.0 | 2026-09-17 | 新增相互独立、与开发板无关的 MS3506V3 和 MS3008V3 C 驱动包及双语文档。 |
| V0.1.0 | 2026-09-16 | 增加 XIAO STM32C5 云台控制、校准工程和协议资料。 |

## 仓库内容

| 目录 | 用途 |
| --- | --- |
| [`lkm-3506v3`](lkm-3506v3) | 独立 MS3506V3 `.c/.h` 驱动，默认 ID `0x141`，范围 `0..345°`。 |
| [`lkm-3008v3`](lkm-3008v3) | 独立 MS3008V3 `.c/.h` 驱动，默认 ID `0x142`，范围 `0..180°`。 |
| [`projects/can_driver_ms`](projects/can_driver_ms) | 带圆形屏 UI 的 XIAO STM32C5 双轴云台参考工程。 |
| [`projects/can_driver_ms_calibration`](projects/can_driver_ms_calibration) | XIAO STM32C5 机械端点与编码器采集工具。 |
| [`docs`](docs) | LK Motor CAN 协议资料。 |

## 跨平台驱动设计

每个电机目录完全独立，只包含以电机命名的头文件、源文件和中英文文档。驱动负责：

- 构造使能、关闭、停止、读取编码器、读取状态和相对位置控制报文。
- 解析 8 字节回复并读取 15-bit 原始编码器值。
- 处理编码器跨零点，并换算为已标定机械角度。
- 检查配置的机械目标范围。
- 使用上层传入的时间戳判断反馈是否仍然有效。

CAN 初始化和波特率、发送队列、中断、线程安全、重试策略、调度、日志及整机安全均由上层负责。因此同一驱动可接入 STM32 HAL、Zephyr、ESP-IDF、Arduino 兼容 CAN 库或其他平台。

## 通用协议

两款电机使用相同的命令布局和操作方式。两个独立驱动包主要用于提供明确的电机命名与各自已测试默认值。

| 项目 | MS3506V3 | MS3008V3 |
| --- | --- | --- |
| 默认 CAN ID | `0x141` | `0x142` |
| 编码器 | 15-bit 磁编码器 | 15-bit 磁编码器 |
| 当前云台机械范围 | `0..345°` | `0..180°` |
| 机械中点 | `172.5°` | `90.0°` |
| 常用总线配置 | 经典 CAN、11-bit ID、8 字节、1 Mbit/s | 经典 CAN、11-bit ID、8 字节、1 Mbit/s |

当前接口支持 `0x80`、`0x81`、`0x88`、`0x90`、`0x9C` 和 `0xA8`。`0xA8` 是增量命令：驱动根据最新有效位置计算一次增量，不能将旧增量当作绝对目标周期重发。

## 驱动接入

选择对应电机目录，将其中 `.c` 和 `.h` 复制到固件工程，并提供经典 CAN 发送回调：

```c
static int board_can_send(void *user, uint16_t id,
                          const uint8_t data[8], size_t length)
{
    return platform_can_send(user, id, data, length);
}
```

开发板 CAN 控制器由上层单独配置。收到 CAN 帧后，将报文与系统运行时间传给对应电机的 `process_frame()`。完整最小示例和 API 表见各驱动目录 README。

## 参考工程

`projects` 目录展示了在 Seeed Studio XIAO STM32C5、PlatformIO 和 Zephyr 上的一种接入方式。云台工程将电机/CAN 状态机放在独立线程中；UI 回调只更新目标，不阻塞等待 CAN。

```powershell
pio run -d projects/can_driver_ms -e seeed-xiao-stm32c5
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

参考工程不是独立驱动的依赖项。

## 协议资料

[LK Motor CAN 协议 V2.3](docs/LK-Motor-CAN-Protocol-V2.3.pdf)

实际测试的 V3 电机在 `0x90` 回复的 `DATA[4:5]` 中返回 15-bit 单圈原始编码器值，因此代码使用该已验证字段计算机械角度。

## 安全说明

- 使能电机前确认供电、共地、CANH/CANL 极性、波特率、ID 和终端电阻。
- 按实际机械结构重新标定编码器方向、中点和换算比例。
- 发送位置命令前必须确认反馈仍然有效。
- 首次测试应远离硬限位，并由上层实现超时和紧急停止路径。
