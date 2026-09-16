# Driver-LK-Motor

本仓库提供基于 PlatformIO/Zephyr 的 LK Motor MS3506V3、MS3008V3 CAN 电机
参考工程，控制器为 Seeed Studio XIAO STM32C5。

English documentation: [README.md](README.md)

## 仓库内容

| 工程 | 用途 | 电机输出 |
| --- | --- | --- |
| [`projects/can_driver_ms`](projects/can_driver_ms) | 带圆形触摸屏界面的双轴云台控制器 | 仅在屏幕请求后使能 |
| [`projects/can_driver_ms_calibration`](projects/can_driver_ms_calibration) | 机械端点和编码器标定工具 | 只读，绝不使能或驱动电机 |

正式控制器默认配置：

- Yaw：MS3506V3，标准 CAN ID `0x141`，机械范围 `0..345°`。
- Pitch：MS3008V3，标准 CAN ID `0x142`，机械范围 `0..180°`。
- 经典 CAN 2.0、11-bit ID、8 字节报文、1 Mbit/s。
- 15-bit 单圈磁编码器反馈。

## 软件架构

正式工程将电机状态机与屏幕任务完全分离：

- 高优先级 10 ms 电机线程负责 CAN 接收、反馈轮询、安全检查、使能/停止和相对
  位置命令。
- 较低优先级 UI 线程读取 CHSC6X 触摸并使用加锁快照刷新 GC9A01 圆形屏幕。
- UI 回调只提交请求或修改目标，不直接调用 CAN，也不等待电机回复。
- CAN 异步发送，每台电机最多保留一个未完成发送帧。

程序通过 `0x90` 读取位置，实测 V3 电机使用 `DATA[4:5]` 的 15-bit 单圈 raw 值。
运动使用 `0xA8` 增量位置闭环；目标变化时只生成一次新命令，不会在反馈未更新时
重复叠加相同增量。

固定机械中心为 Yaw `172.5°`、Pitch `90°`，滑条目标只是临时运动目标，不会
改写机械中心。实测 MS3506 Yaw 在目标附近持续保持时会出现 PID 高频抖动，因此
到位后发送 `0x81` 停止；MS3008 Pitch 保持使能，以承托垂直负载。

## 编译

安装 PlatformIO 和 Seeed Studio XIAO 平台后执行：

```powershell
pio run -d projects/can_driver_ms -e seeed-xiao-stm32c5
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

UF2 位于各工程的 `.pio/build/seeed-xiao-stm32c5/`。构建目录不提交到版本库。

## 标定流程

1. 烧录只读标定工程。
2. 按屏幕提示，手动将两轴移动到最小、近似中心和最大位置，采集共六个点。
3. 优先采用稳定端点计算得到的环形数学中点，不使用误差较大的手动中心。
4. 将中心 raw、方向和每度计数写入
   `projects/can_driver_ms/include/app_config.h`。
5. 重新编译；首次运动测试时确保两轴远离机械限位。

仓库中的默认映射来自两次完整端点采样。具体参数及保护逻辑见各工程说明。

## 协议资料

- [LK Motor CAN 协议 V2.3](docs/LK-Motor-CAN-Protocol-V2.3.pdf)

该 PDF 属于较早版本的 LK 协议资料，可用于核对命令含义和基础帧格式；实测 V3
电机采用 15-bit raw 编码值，offset 字段也不满足旧版固定零偏关系，因此代码中
明确采用经过实机验证的 V3 解析方式。

## 安全提示

- 运动前确认电机供电、共地、CANH/CANL、波特率和 ID。
- 正确连接两个终端的断电 CAN 总线，CANH 与 CANL 间通常约为 60Ω。
- 首次测试时让两轴远离机械限位。
- 对未知机械安装，建议先使用只读标定工程；它不会发送使能或运动命令。
