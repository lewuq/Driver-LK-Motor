# Driver-LK-Motor

PlatformIO/Zephyr reference projects for controlling LK Motor MS3506V3 and
MS3008V3 CAN motors with a Seeed Studio XIAO STM32C5.

Chinese documentation: [README_CN.md](README_CN.md)

## Repository contents

| Project | Purpose | Motor output |
| --- | --- | --- |
| [`projects/can_driver_ms`](projects/can_driver_ms) | Two-axis gimbal controller with round-display UI | Enabled only after an on-screen request |
| [`projects/can_driver_ms_calibration`](projects/can_driver_ms_calibration) | Read-only mechanical endpoint and encoder calibration tool | Never enables or moves a motor |

The controller is configured for:

- MS3506V3 yaw motor at standard CAN ID `0x141`, mechanical range `0..345°`.
- MS3008V3 pitch motor at standard CAN ID `0x142`, mechanical range `0..180°`.
- Classic CAN 2.0, 11-bit identifiers, eight-byte frames, 1 Mbit/s.
- 15-bit single-turn magnetic encoder feedback.

## Architecture

The production controller separates the motor state machine from the UI:

- A high-priority 10 ms motor thread owns CAN receive processing, feedback
  polling, safety checks, enable/stop commands, and relative position commands.
- The lower-priority UI thread reads the CHSC6X touch controller and renders the
  GC9A01 round display from a locked snapshot.
- UI callbacks only queue requests or update targets. They never wait for CAN or
  call the motor transport directly.
- CAN transmission is asynchronous and limited to one outstanding frame per
  motor.

Position feedback comes from command `0x90`, using `DATA[4:5]` as the tested V3
15-bit raw single-turn value. Motion uses `0xA8` relative position closed-loop
control. A target change produces one new relative command; the same increment
is not repeatedly transmitted without updated feedback.

The fixed mechanical center is yaw `172.5°` and pitch `90°`. Slider targets are
transient and never redefine this home position. MS3506 yaw is stopped with
`0x81` after reaching a target to avoid observed near-target holding-PID
hunting. MS3008 pitch remains enabled to hold the vertical load.

## Build

Install PlatformIO with the Seeed Studio XIAO platform, then run:

```powershell
pio run -d projects/can_driver_ms -e seeed-xiao-stm32c5
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

Generated UF2 files are placed under each project's
`.pio/build/seeed-xiao-stm32c5/` directory. Build directories are intentionally
excluded from version control.

## Calibration workflow

1. Flash the read-only calibration project.
2. Move each axis by hand to its minimum, approximate center, and maximum when
   prompted, then capture the six points.
3. Prefer the derived circular midpoint calculated from repeatable endpoints
   over the manually positioned center.
4. Apply the resulting center, direction, and counts-per-degree values to
   `projects/can_driver_ms/include/app_config.h`.
5. Rebuild and test with both axes away from hard stops.

The checked-in mapping is based on two complete endpoint runs. See the project
documentation for the exact values and safety behavior.

## Protocol reference

- [LK Motor CAN Protocol V2.3](docs/LK-Motor-CAN-Protocol-V2.3.pdf)

The bundled protocol document describes an earlier LK protocol generation.
Command meanings and frame layouts are useful references, but the tested V3
motors use 15-bit raw encoder data and do not exhibit the older fixed-offset
relationship. The implementation therefore documents the verified V3 parsing
behavior explicitly.

## Safety notes

- Verify motor power, common ground, CANH/CANL polarity, bitrate, and IDs before
  allowing motion.
- A correctly terminated, unpowered two-endpoint CAN bus normally measures
  approximately 60 ohms between CANH and CANL.
- Keep both axes away from mechanical stops during initial tests.
- The calibration project is the appropriate first firmware for an unknown
  mechanical installation because it never sends enable or motion commands.
