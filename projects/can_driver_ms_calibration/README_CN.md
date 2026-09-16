# LK 云台只读标定工具

该工程用于在 XIAO STM32C5 上测量 MS3506V3 Yaw 与 MS3008V3 Pitch 安装后的
机械范围和编码器映射，是正式控制工程的配套工具。

English documentation: [README.md](README.md)

## 安全约束

本工程对电机控制严格只读，只发送编码器读取命令 `0x90`，不发送电机使能、
停止、力矩、速度或位置命令。所有轴均由操作者手动移动。

源码明确不使用 `0x88`、`0xA4`、`0xA8` 等运动命令。标定前应确保机构能够
安全地手动移动，禁止用力越过机械限位。

## 总线配置

- 经典 CAN 2.0，1 Mbit/s。
- 标准 11-bit ID，8 字节报文。
- MS3506V3 Yaw：`0x141`。
- MS3008V3 Pitch：`0x142`。
- 从 `0x90 DATA[4:5]` 读取 15-bit raw 编码值。

## 六点采集流程

圆形屏幕依次提示：

1. Yaw 机械最小位置。
2. Yaw 近似中心位置。
3. Yaw 机械最大位置。
4. Pitch 机械最小位置。
5. Pitch 近似中心位置。
6. Pitch 机械最大位置。

每一步都应缓慢手动移动指定轴，等待 raw 值稳定后只按一次 **CAPTURE**。不要
依靠撞击限位寻找位置。

## 结果计算

15-bit 编码器在 32768 处回绕。工具会检查两个环形方向，选择满足
最小 → 中心 → 最大关系的方向，并打印：

- 最小、手动中心、最大和端点推导中心 raw。
- 方向（`+1` 或 `-1`）。
- 环形总跨度和每度计数。
- 手动中心相对数学中点的误差。
- 可复制的 `CAL CONFIG` 配置。

机械端点通常比人工寻找视觉中心更稳定，因此最终配置使用最小、最大端点推导的
环形数学中点；手动中心只用于检查误差。

控制工程使用的换算关系为：

```text
delta_counts = wrapped(raw - center_raw)
angle_deg = center_deg + direction * delta_counts / counts_per_deg
```

建议至少完整采集两次。若两次端点只相差几个 count，其可信度远高于单次手动
中心结果。

## 编译

在仓库根目录执行：

```powershell
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

输出：`.pio/build/seeed-xiao-stm32c5/firmware.uf2`。

## 应用标定结果

把推导得到的 `CENTER_RAW`、`DIRECTION` 和 `COUNTS_PER_DEG` 写入
`../can_driver_ms/include/app_config.h`。确认机械硬限位和 UI 工作范围后，再启用
`GIMBAL_RELATIVE_CALIBRATION_VALID`。

修改配置后必须重新编译正式控制器。首次运动测试应让两轴远离机械限位，并保留
完整串口日志。

## 关键文件

- `src/calibration.c`：采集状态机和环形映射计算。
- `src/lkm_motor.c`：只读 CAN 传输和 V3 编码器解析。
- `src/main.c`：CAN 初始化、轮询、诊断和 UI 主循环。
- `src/round_ui.c`：采集提示和实时 raw 显示。
- `include/app_config.h`：总线 ID 和已知机械行程。

协议资料：[LK Motor CAN 协议 V2.3](../../docs/LK-Motor-CAN-Protocol-V2.3.pdf)。
