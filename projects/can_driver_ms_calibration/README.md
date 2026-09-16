# Read-only LK Gimbal Calibration

Companion calibration firmware for measuring the installed mechanical ranges of
MS3506V3 yaw and MS3008V3 pitch motors on XIAO STM32C5.

Chinese documentation: [README_CN.md](README_CN.md)

## Safety contract

This project is deliberately read-only with respect to motor control. It sends
only encoder command `0x90` and never sends motor enable, stop, torque, speed, or
position commands. The operator moves each axis by hand.

The source explicitly excludes `0x88`, `0xA4`, `0xA8`, and other motion
commands. Keep the motors mechanically unloaded enough to move safely by hand.

## Bus configuration

- Classic CAN 2.0 at 1 Mbit/s.
- Standard 11-bit identifiers and eight-byte frames.
- MS3506V3 yaw at `0x141`.
- MS3008V3 pitch at `0x142`.
- 15-bit raw encoder value from `0x90 DATA[4:5]`.

## Capture sequence

The round display guides the operator through six captures:

1. Yaw mechanical minimum.
2. Yaw approximate center.
3. Yaw mechanical maximum.
4. Pitch mechanical minimum.
5. Pitch approximate center.
6. Pitch mechanical maximum.

At each step, move the requested axis slowly by hand, let the displayed raw
value settle, and press **CAPTURE** once. Do not force an axis beyond its physical
stop.

## Result calculation

Encoder counts wrap at 32768. The tool evaluates both circular directions,
selects the direction consistent with minimum → center → maximum, and prints:

- Minimum, manually captured center, maximum, and derived center raw counts.
- Direction (`+1` or `-1`).
- Total circular span and counts per degree.
- Manual-center error relative to the endpoint-derived midpoint.
- Ready-to-copy `CAL CONFIG` values.

Mechanical endpoints are normally more repeatable than manually positioning the
axis at visual center. The generated configuration therefore uses the circular
midpoint derived from minimum and maximum; the captured center is retained as a
diagnostic check.

The mapping used by the controller is:

```text
delta_counts = wrapped(raw - center_raw)
angle_deg = center_deg + direction * delta_counts / counts_per_deg
```

Run the full six-point sequence at least twice. Endpoint differences of only a
few counts provide much stronger evidence than a single manual center capture.

## Build

From the repository root:

```powershell
pio run -d projects/can_driver_ms_calibration -e seeed-xiao-stm32c5
```

Output: `.pio/build/seeed-xiao-stm32c5/firmware.uf2`.

## Applying the result

Copy the derived `CENTER_RAW`, `DIRECTION`, and `COUNTS_PER_DEG` values into
`../can_driver_ms/include/app_config.h`. Verify hard limits and UI working ranges
before enabling `GIMBAL_RELATIVE_CALIBRATION_VALID`.

The controller must be rebuilt after configuration changes. Begin the first
motion test away from both hard stops and retain the serial log.

## Important files

- `src/calibration.c`: capture state machine and circular mapping calculation.
- `src/lkm_motor.c`: read-only CAN transport and V3 encoder decoding.
- `src/main.c`: CAN setup, polling, diagnostics, and UI loop.
- `src/round_ui.c`: capture prompts and live raw-value display.
- `include/app_config.h`: bus IDs and known mechanical travel.

Protocol reference: [LK Motor CAN Protocol V2.3](../../docs/LK-Motor-CAN-Protocol-V2.3.pdf).
