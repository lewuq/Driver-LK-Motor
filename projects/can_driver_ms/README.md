# Dual LK Gimbal Controller

Production-oriented two-axis gimbal controller for XIAO STM32C5, MS3506V3
yaw, MS3008V3 pitch, and the Seeed Studio round display/touch hardware.

Chinese documentation: [README_CN.md](README_CN.md)

## Fixed configuration

| Axis | Motor | CAN ID | Hard range | UI range | Home |
| --- | --- | ---: | ---: | ---: | ---: |
| Yaw | MS3506V3 | `0x141` | `0..345°` | `1..344°` | `172.5°` |
| Pitch | MS3008V3 | `0x142` | `0..180°` | `1..175°` | `90°` |

The bus uses classic CAN 2.0 at 1 Mbit/s with 11-bit identifiers and eight-byte
frames. The firmware does not scan bitrate or node IDs.

## Runtime design

The application uses two execution contexts:

1. **Motor thread** — high-priority 10 ms polling loop. It drains the CAN RX
   queue, advances the gimbal state machine, sends read/enable/stop/position
   frames, checks limits, and reports communication faults.
2. **UI thread** — reads CHSC6X touch and refreshes the GC9A01 display. It only
   queues start/calibration requests and slider targets. Rendering uses a locked
   snapshot, so slow display transfers cannot block motor control.

No application interrupt handler implements motor control. The Zephyr CAN
driver may use hardware interrupts internally, while application logic remains
a normal non-blocking thread/state machine.

## State machine

| State | Behavior |
| --- | --- |
| `DISARM` | Motors remain stopped; low-rate `0x90` reads verify both nodes. |
| `CHECK` | Discards cached positions and requires fresh post-request feedback. |
| `CENTER` | Enables both axes and sends one low-speed relative `0xA8` command. |
| `READY` | Sliders are active; motion is sent only when a target changes. |
| `FAULT` | Sends stop commands and latches a visible error condition. |

Every START or CALIBRATE request restores the immutable home targets of
`172.5°/90°`. Slider movement cannot redefine mechanical home.

## Position and anti-jitter behavior

The implementation reads the tested V3 15-bit raw encoder from `0x90`
`DATA[4:5]`. It converts the circular count into mechanical degrees using:

- Yaw: center raw `12178`, direction `+1`, `91.8087 counts/°`.
- Pitch: center raw `8445`, direction `+1`, `93.4083 counts/°`.

`0xA8` receives the signed difference between requested and current mechanical
angle. It is an incremental command, so it is never blindly retransmitted.

MS3506 yaw exhibited severe high-frequency holding-PID hunting near `172°`.
After yaw reaches a target within tolerance, the state machine sends `0x81 STOP`.
A later yaw slider change first sends `0x88`, waits for asynchronous TX
completion, then sends one new `0xA8`. MS3008 pitch remains closed-loop enabled
at rest because it supports the vertical load.

## Safety mechanisms

- No motion until both axes return fresh `0x90` data inside hard limits.
- Startup feedback must be no more than one second old.
- Runtime feedback watchdog, CAN TX error reporting, and per-axis RX counters.
- Centering divergence guard stops both motors when error grows by more than 3°.
- Centering timeout and one-degree feedback allowance for endpoint capture
  spread/encoder quantization.
- One outstanding asynchronous CAN frame per motor prevents TX FIFO flooding.
- Conservative UI ranges keep commanded targets away from physical stops.

## User interface

- Two sliders show target and measured yaw/pitch angles.
- The action button requests initial centering or a new centering cycle.
- Fault text is red and the button changes color while pressed.
- Theme colors: `#8FC31F` and `#004966`.
- Backlight PWM is set to 80%.

## Build

From the repository root:

```powershell
pio run -d projects/can_driver_ms -e seeed-xiao-stm32c5
```

Output: `.pio/build/seeed-xiao-stm32c5/firmware.uf2`.

## Important files

- `include/app_config.h`: IDs, limits, calibrated mapping, speeds, and timing.
- `src/lkm_motor.c`: classic CAN transport and V3 feedback conversion.
- `src/gimbal.c`: motor state machine, safety policy, and target lifecycle.
- `src/main.c`: thread separation, CAN setup, and UI snapshots.
- `src/round_ui.c`: display rendering and event generation.
- `src/touch.c`: CHSC6X polling.

Protocol reference: [LK Motor CAN Protocol V2.3](../../docs/LK-Motor-CAN-Protocol-V2.3.pdf).
