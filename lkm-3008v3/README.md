# MS3008V3 Portable CAN Driver

**🌐 [English](README.md) | [中文](README_CN.md)**

MS3008V3 Portable CAN Driver is a small, board-independent C interface for the
15-bit classic-CAN version of the LK Motor MS3008V3. It builds and parses the
motor's eight-byte protocol frames while leaving the CAN peripheral, scheduler,
interrupts, queues, and logging to the application.

## Version history

| Version | Updated | Highlights |
| --- | --- | --- |
| V0.1.0 | 2026-09-17 | Initial portable command, feedback, encoder conversion, and callback interface. |

## Features

- Pure C99 interface with no RTOS or MCU SDK dependency.
- User-supplied CAN transmit callback.
- Standard 11-bit identifier and classic eight-byte frames.
- Enable (`0x88`), disable (`0x80`), stop (`0x81`), encoder (`0x90`), state (`0x9C`), and relative position (`0xA8`) commands.
- 15-bit raw encoder parsing and wrap-aware mechanical-angle conversion.
- Configurable CAN ID, limits, direction, center, and counts per degree.
- Feedback age helper for application-level safety checks.

## Default configuration

| Item | Default |
| --- | --- |
| Motor | MS3008V3 CAN, 15-bit encoder |
| CAN ID | `0x142` |
| Bus format | Classic CAN, standard ID, 8 data bytes |
| Typical bitrate | 1 Mbit/s, configured by the application |
| Mechanical range | `0..180°` |
| Calibrated center | raw `8445` at `90.0°` |
| Scale | `93.4083 counts/°` |

The checked-in mechanical mapping belongs to the tested gimbal. Recalibrate it
when the motor, mounting orientation, or mechanical assembly changes.

## Integration

Copy `lkm_3008v3.c` and `lkm_3008v3.h` into the target firmware. Implement one
callback that submits a classic CAN frame through the board's CAN driver:

```c
static int board_can_send(void *user, uint16_t id,
                          const uint8_t data[8], size_t length)
{
    /* Call STM32 HAL, Zephyr, ESP-IDF, Arduino, or another CAN API here. */
    return platform_can_send(user, id, data, length);
}

struct lkm_3008v3 motor;
struct lkm_3008v3_config config;

lkm_3008v3_default_config(&config, board_can_send, can_handle);
lkm_3008v3_init(&motor, &config);
lkm_3008v3_read_encoder(&motor);
```

The callback must copy the eight data bytes before it returns if the platform
queues transmission asynchronously.

Pass each received standard CAN frame to the parser:

```c
lkm_3008v3_process_frame(&motor, rx_id, rx_data, rx_length, uptime_ms);
```

After a valid `0x90` reply, `motor.angle_deg` and the encoder fields are
available. `lkm_3008v3_set_position()` requires this feedback before it can turn a
mechanical target into a safe `0xA8` relative command.

## API summary

| Function | Purpose |
| --- | --- |
| `lkm_3008v3_default_config()` | Load tested defaults and attach the send callback. |
| `lkm_3008v3_init()` | Validate configuration and initialize runtime state. |
| `lkm_3008v3_enable()` / `disable()` / `stop()` | Build and send basic motor commands. |
| `lkm_3008v3_read_encoder()` | Request 15-bit raw position with command `0x90`. |
| `lkm_3008v3_read_state()` | Request state feedback with command `0x9C`. |
| `lkm_3008v3_set_position()` | Send a bounded target as a relative `0xA8` move. |
| `lkm_3008v3_process_frame()` | Parse a received frame and update motor state. |
| `lkm_3008v3_feedback_fresh()` | Check application-provided feedback timestamps. |

## Protocol reference

[LK Motor CAN Protocol V2.3](../docs/LK-Motor-CAN-Protocol-V2.3.pdf)

## Safety

The driver validates arguments and configured position limits, but it does not
own the bus or machine safety state. The application must verify current
feedback, power, grounding, bitrate, CAN wiring, timeouts, and mechanical limits
before enabling motion. Do not retransmit an old `0xA8` increment repeatedly.
