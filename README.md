# Driver-LK-Motor

**🌐 [English](README.md) | [中文](README_CN.md)**

Driver-LK-Motor provides portable C drivers and PlatformIO/Zephyr reference
projects for the 15-bit classic-CAN versions of LK Motor MS3506V3 and MS3008V3.
The standalone drivers are independent of the MCU, CAN peripheral, RTOS, and UI;
applications connect them through one transmit callback.

## Version history

| Version | Updated | Highlights |
| --- | --- | --- |
| V0.2.0 | 2026-09-17 | Added independent, board-neutral MS3506V3 and MS3008V3 C driver packages with bilingual documentation. |
| V0.1.0 | 2026-09-16 | Added the XIAO STM32C5 gimbal controller, calibration project, and protocol reference. |

## Repository contents

| Directory | Purpose |
| --- | --- |
| [`lkm-3506v3`](lkm-3506v3) | Standalone MS3506V3 `.c/.h` driver, default ID `0x141`, range `0..345°`. |
| [`lkm-3008v3`](lkm-3008v3) | Standalone MS3008V3 `.c/.h` driver, default ID `0x142`, range `0..180°`. |
| [`projects/can_driver_ms`](projects/can_driver_ms) | XIAO STM32C5 two-axis gimbal reference application with round-display UI. |
| [`projects/can_driver_ms_calibration`](projects/can_driver_ms_calibration) | XIAO STM32C5 mechanical endpoint and encoder capture utility. |
| [`docs`](docs) | LK Motor CAN protocol reference. |

## Portable driver design

Each motor directory is self-contained and contains only four files: a
motor-named header, a motor-named source file, and English/Chinese documentation.
The driver is responsible for:

- Building classic CAN commands for enable, disable, stop, encoder read, state
  read, and relative position control.
- Parsing eight-byte responses and decoding 15-bit raw encoder feedback.
- Converting raw position to a calibrated mechanical angle across encoder wrap.
- Enforcing configured mechanical target limits.
- Reporting feedback freshness from timestamps supplied by the application.

The application remains responsible for CAN initialization and bitrate,
transmit queues, interrupts, thread safety, retry policy, scheduling, logging,
and machine-level safety. This keeps the same driver usable with STM32 HAL,
Zephyr, ESP-IDF, Arduino-compatible CAN libraries, or another platform.

## Common protocol

Both motors use the same command layouts and public operation pattern. Their
separate packages mainly provide motor-specific names and tested defaults.

| Item | MS3506V3 | MS3008V3 |
| --- | --- | --- |
| Default CAN ID | `0x141` | `0x142` |
| Encoder | 15-bit magnetic | 15-bit magnetic |
| Mechanical range in this gimbal | `0..345°` | `0..180°` |
| Mechanical center | `172.5°` | `90.0°` |
| Typical bus | Classic CAN, 11-bit ID, 8-byte frame, 1 Mbit/s | Classic CAN, 11-bit ID, 8-byte frame, 1 Mbit/s |

Supported commands are `0x80`, `0x81`, `0x88`, `0x90`, `0x9C`, and `0xA8`.
`0xA8` is incremental: the libraries derive one increment from the latest
verified position and must not be used as a periodically retransmitted absolute
setpoint.

## Driver integration

Choose one motor package, copy its `.c` and `.h` into the firmware, and provide
a classic-CAN transmit callback:

```c
static int board_can_send(void *user, uint16_t id,
                          const uint8_t data[8], size_t length)
{
    return platform_can_send(user, id, data, length);
}
```

Configure the board's CAN controller separately. Pass received frames and the
platform uptime to the motor-specific `process_frame()` function. See each
driver's README for a complete minimal example and API table.

## Reference projects

The `projects` directory demonstrates one integration on Seeed Studio XIAO
STM32C5 using PlatformIO and Zephyr. The gimbal application keeps motor/CAN work
in an independent state-machine thread; UI callbacks only update targets and do
not block on CAN communication.

```powershell
pio run -d projects/can_driver_ms -e seeed-xiao-stm32c5
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

These projects are examples rather than dependencies of the portable drivers.

## Protocol reference

[LK Motor CAN Protocol V2.3](docs/LK-Motor-CAN-Protocol-V2.3.pdf)

The V3 motors tested here return the 15-bit raw single-turn encoder in `0x90`
`DATA[4:5]`; the implementation uses that verified field for mechanical angle.

## Safety

- Confirm motor supply, common ground, CANH/CANL polarity, bitrate, IDs, and
  termination before enabling either motor.
- Calibrate encoder direction, center, and scale for the actual mechanism.
- Require fresh position feedback before sending a position command.
- Keep initial tests away from hard stops and implement an application-level
  timeout and emergency-stop path.
