# MS3008V3 通用 CAN 驱动

**🌐 [English](README.md) | [中文](README_CN.md)**

MS3008V3 通用 CAN 驱动是面向 LK Motor MS3008V3 15-bit 经典 CAN 版本的轻量纯 C 接口。驱动负责构造和解析 8 字节电机协议报文；CAN 外设、中断、任务、队列、锁和日志均由上层工程实现。

## 版本迭代

| 版本 | 更新日期 | 主要内容 |
| --- | --- | --- |
| V0.1.0 | 2026-09-17 | 首次提供跨平台命令、反馈解析、编码器换算和发送回调接口。 |

## 功能特性

- 纯 C99，不依赖 RTOS 或特定 MCU SDK。
- 通过用户回调接入任意开发板的 CAN 发送接口。
- 使用标准 11-bit ID 和 8 字节经典 CAN 帧。
- 支持使能 `0x88`、关闭 `0x80`、停止 `0x81`、编码器 `0x90`、状态 `0x9C` 和相对位置 `0xA8`。
- 支持 15-bit 原始编码器解析和跨零点角度换算。
- CAN ID、机械限位、方向、中点和比例均可配置。
- 提供反馈时效检查，便于上层实现超时保护。

## 默认配置

| 项目 | 默认值 |
| --- | --- |
| 电机 | MS3008V3 CAN，15-bit 编码器 |
| CAN ID | `0x142` |
| 总线格式 | 经典 CAN、标准 ID、8 字节数据 |
| 常用波特率 | 1 Mbit/s，由上层配置 |
| 机械范围 | `0..180°` |
| 已标定中点 | raw `8445` 对应 `90.0°` |
| 换算比例 | `93.4083 counts/°` |

仓库中的机械映射来自已测试云台。更换电机、安装方向或机械结构后应重新标定。

## 接入方法

将 `lkm_3008v3.c` 和 `lkm_3008v3.h` 复制到目标工程，并实现一个经典 CAN 发送回调：

```c
static int board_can_send(void *user, uint16_t id,
                          const uint8_t data[8], size_t length)
{
    /* 在这里调用 STM32 HAL、Zephyr、ESP-IDF、Arduino 或其他 CAN API。 */
    return platform_can_send(user, id, data, length);
}

struct lkm_3008v3 motor;
struct lkm_3008v3_config config;

lkm_3008v3_default_config(&config, board_can_send, can_handle);
lkm_3008v3_init(&motor, &config);
lkm_3008v3_read_encoder(&motor);
```

如果平台采用异步发送队列，回调必须在返回前复制这 8 字节数据。

收到标准 CAN 帧后交给解析函数：

```c
lkm_3008v3_process_frame(&motor, rx_id, rx_data, rx_length, uptime_ms);
```

收到有效 `0x90` 回复后，可读取 `motor.angle_deg` 和编码器字段。`lkm_3008v3_set_position()` 必须先获得有效位置反馈，才能将机械目标换算为 `0xA8` 相对运动报文。

## API 概览

| 函数 | 用途 |
| --- | --- |
| `lkm_3008v3_default_config()` | 载入已测试默认值并绑定发送回调。 |
| `lkm_3008v3_init()` | 校验配置并初始化运行状态。 |
| `lkm_3008v3_enable()` / `disable()` / `stop()` | 构造并发送基础电机命令。 |
| `lkm_3008v3_read_encoder()` | 使用 `0x90` 请求 15-bit 原始位置。 |
| `lkm_3008v3_read_state()` | 使用 `0x9C` 请求状态反馈。 |
| `lkm_3008v3_set_position()` | 将限位内目标作为 `0xA8` 相对运动发送。 |
| `lkm_3008v3_process_frame()` | 解析接收帧并更新电机状态。 |
| `lkm_3008v3_feedback_fresh()` | 检查由上层传入的反馈时间戳。 |

## 协议资料

[LK Motor CAN 协议 V2.3](../docs/LK-Motor-CAN-Protocol-V2.3.pdf)

## 安全说明

驱动会检查参数和配置限位，但不负责整机安全状态。上层在允许运动前必须检查最新反馈、电源、共地、波特率、CAN 接线、通信超时和机械限位。不要反复重发旧的 `0xA8` 增量命令。
