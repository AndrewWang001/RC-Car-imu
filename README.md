# RC Car with IMU

ESP32 firmware for an RC car. It takes radio receiver inputs, reads an IMU and drives the
ESCs and steering servo, with gyro-based steering control on top of normal RC driving.

Written in C with ESP-IDF 5.5 and FreeRTOS.

## Features

- **RC input:** receiver PWM pulses measured with GPIO edge interrupts, plus a 3-sample
  median filter to reject glitched pulses. Calibrated min/center/max with deadbands.
- **Outputs:** 50 Hz PWM (LEDC) for two ESCs and the steering servo.
- **IMU (MPU-6050 over I²C, 100 Hz):**
  - startup gyro calibration with a robust average (median/MAD outlier rejection, retried
    if the car moves)
  - per-axis Kalman filter tracking the rate and the gyro bias, with zero-rate updates
    when the car is still
  - complementary filter for roll and pitch, and integrated yaw for heading
- **Drive modes:**
  - `manual`: the stick goes straight to the servo
  - `assist`: heading hold. When the steering stick is centered, a PID holds the heading
    captured at release, so the car drives straight.
  - `rate`: yaw rate control. The stick sets a target turn rate, and stick feedforward plus a
    PID on the gyro tracks it.
- **PID controller:** derivative on measurement (taken from the gyro) and anti-windup.
- **Safety:**
  - receiver signal-loss failsafe
  - arming only with the throttle at neutral
  - tip-over throttle cut
  - throttle ramp limiting (braking stays instant)
  - closed-loop steering disabled in reverse or when the IMU data is stale
- **Serial console:** change the mode and PID gains live, save them to flash (NVS), and
  stream telemetry as a status line or as CSV for plotting.

## Hardware

- ESP32 DevKit (ESP32-WROOM-32)
- RC receiver (PWM outputs)
- 2x ESCs driven with the same signal
- Steering servo (500-2500 us)
- MPU-6050 IMU, mounted flat

| Function | GPIO |
|---|---|
| Receiver CH1 (steering) | 34 |
| Receiver CH2 (throttle) | 35 |
| ESC 1 | 25 |
| ESC 2 | 26 |
| Steering servo | 27 |
| IMU SDA | 21 |
| IMU SCL | 22 |
| Status LED | 2 |

IMU power: VCC to 3V3, GND to GND. All pins and receiver calibration values are in
[`main/config.h`](main/config.h).

## Code layout

| File | What it does |
|---|---|
| `main.c` | Startup |
| `config.h` | Pins, receiver calibration, output limits, task priorities |
| `rc_input.c` | Receiver pulse capture (ISR), filtering, failsafe timeout |
| `actuators.c` | LEDC PWM for the ESCs and servo |
| `imu.c` | MPU-6050 driver and attitude estimation |
| `pid.c` | PID controller |
| `control.c` | 50 Hz control loop: modes, safety state machine, outputs |
| `params.c` | Tunable parameters, saved to NVS |
| `console.c` | Serial commands and telemetry |

| Task | Core | Priority | Rate |
|---|---|---|---|
| imu | 1 | 6 | 100 Hz |
| control | 1 | 5 | 50 Hz |
| console | 0 | 3 | on input |
| telemetry | 0 | 2 | 5 Hz status / 50 Hz CSV |

## Building

```
idf.py build flash monitor
```

## Using it

1. Power on with the car still and leave it for about 2 s while the gyro calibrates.
2. Center the throttle stick to arm.
3. The LED shows the state:
   - solid: driving
   - slow blink: not armed, or the IMU is calibrating
   - fast blink: failsafe or tipped

### Serial commands (115200 baud)

```
mode manual|assist|rate     drive mode
set <name> <value>          change a parameter
get                         list parameters
save                        save to flash
defaults                    restore defaults
log off|status|csv          telemetry output
imu                         IMU details
```

### Setup and tuning

1. **Yaw direction:** run `log status` and turn the car right by hand. `yaw` should go
   positive. If it goes negative, run `set yaw_sign -1`.
2. **Heading hold (`mode assist`):**
   1. Start with `set hold_ki 0` and `set hold_kd 0`.
   2. Raise `hold_kp` until the car corrects firmly without weaving.
   3. Add `hold_kd` to damp the oscillation.
   4. Add a little `hold_ki` if it still drifts to one side.
3. **Rate mode (`mode rate`):**
   1. Set `max_rate` to how fast the car turns at full lock.
   2. Tune `rate_kp`, then `rate_ki`.
4. `save` once the car drives well.
5. To record a run, use `log csv` and save the serial output to a file. Plot `yaw_rate`
   against `target` to compare the controller response to the target.
