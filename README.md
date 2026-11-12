# RC Car IMU

ESP32 firmware for my RC car. It reads the RC receiver and an MPU-6050 IMU, and drives two ESCs
and the steering servo. The gyro is used to help the steering.

ESP32 + ESP-IDF 5.5 + FreeRTOS.

## Hardware

- ESP32 DevKit
- RC receiver (PWM)
- 2 ESCs on the same signal
- Steering servo
- MPU-6050, mounted flat

## Pinout

| Function | GPIO |
|---|---|
| Receiver CH1 (steering) | 34 |
| Receiver CH2 (throttle) | 35 |
| ESC 1 | 25 |
| ESC 2 | 26 |
| Servo | 27 |
| IMU SDA | 21 |
| IMU SCL | 22 |
| LED | 2 |

IMU VCC goes to 3V3. Pins and receiver calibration are in `main/config.h`.

## Drive modes

- `manual`: normal RC steering
- `assist`: when the steering stick is centered, a PID on the gyro keeps the car going straight
- `rate`: the stick sets how fast the car turns, and the gyro corrects it to match

The car only arms with the throttle at neutral. It cuts the throttle if the signal is lost or the
car flips over.

LED: solid = driving, slow blink = not armed or calibrating, fast blink = signal lost or flipped.

## Building

```
idf.py build flash monitor
```

Keep the car still for about 2 s after powering on so the gyro can calibrate.

## Serial commands

```
mode manual|assist|rate
set <name> <value>
get
save
defaults
log off|status|csv
imu
```

Settings are saved to flash with `save`.

## Tuning

1. Turn the car right by hand. `yaw` should go positive. If not, run `set yaw_sign -1`.
2. For assist mode, start with only `hold_kp` and raise it until the car corrects without
   weaving. Then add a bit of `hold_kd`, and a bit of `hold_ki` if it still drifts.
3. `save` when it drives well.

`log csv` prints data at 50 Hz if you want to plot it.
