# Reaction Wheel Balancing System

This project is a one-dimensional reaction wheel balancing system built with an
ESP32, an MPU6050 IMU, an L298N motor driver, and a bidirectional 12 V brushed
DC motor. The controller estimates the body pitch angle and drives a reaction
wheel to hold the system near the upright target angle:

```text
target pitch = 1.200 rad
```

The current firmware is ready for constrained hardware testing and presentation.
It includes sensor calibration, pitch filtering, PID control, signed PWM motor
control with motor deadband handling, and a recovery lockout that prevents the
motor from fighting when the rig is too far from the balance point.

## System Overview

```text
MPU6050 accelerometer + gyro
        |
        v
Calibration and unit conversion
        |
        v
1D Kalman pitch estimate
        |
        v
Pitch error from pi / 2
        |
        v
Recovery lockout check
        |
        v
PID controller
        |
        v
Signed PWM command
        |
        v
L298N motor driver
        |
        v
Reaction wheel torque
```

The control loop runs at:

```text
200 Hz, Ts = 0.005 s
```

Serial debug output runs at:

```text
10 Hz
```

## Hardware

| Component | Details |
| --- | --- |
| Microcontroller | ESP32 development board |
| IMU | MPU6050 |
| Motor driver | L298N |
| Motor | 12 V bidirectional brushed DC motor |
| Motor no-load speed | `550 rpm` |
| Motor reduction ratio | `1:9` |
| Motor rated torque | `0.7 kg*cm = 0.0686 N*m` |
| Motor rated current | `0.5 A` |
| Reaction wheel mass | `0.046 kg` |
| Reaction wheel radius | `0.0425 m` |
| Reaction wheel moment of inertia | `0.0000415 kg*m^2` |
| Body moment of inertia | `0.000375 kg*m^2` |
| Total body mass with wheel | `0.271 kg` |
| COM distance to pivot | `0.0517 m` |

## Wiring

| ESP32 Pin | Connection |
| --- | --- |
| GPIO 21 | MPU6050 SDA |
| GPIO 22 | MPU6050 SCL |
| GPIO 25 | L298N ENA / PWM |
| GPIO 26 | L298N IN1 |
| GPIO 27 | L298N IN2 |
| GND | Common ground with L298N and 12 V supply |

The ESP32, L298N, and bench supply must share a common ground. The L298N motor
supply should be powered from the 12 V bench supply.

## Direction Convention

The measured motor orientation assumes the motor label is at the bottom.

| Command | L298N State | Wheel Motion | System Motion |
| --- | --- | --- | --- |
| Positive PWM | `IN1 HIGH`, `IN2 LOW` | Clockwise | Sends wheel/body right |
| Negative PWM | `IN1 LOW`, `IN2 HIGH` | Counterclockwise | Sends wheel/body left |
| Zero PWM | `IN1 LOW`, `IN2 LOW`, PWM `0` | Off/coast | No drive |

Pitch increases as the wheel/body tilts to the right.

```text
pitch < pi / 2  -> negative command -> counterclockwise wheel spin
pitch > pi / 2  -> positive command -> clockwise wheel spin
```

If the IMU or motor wiring is changed, adjust these constants in `src/Main.cpp`:

```cpp
const float angleSign = 1.0f;
const float gyroPitchSign = 1.0f;
const int motorSign = 1;
```

## Pitch Calibration

Measured mounted pitch range:

```text
left mechanical limit  = 0.566 rad
right mechanical limit = 2.58 rad
target pitch           = pi / 2 rad = 1.571 rad
```

The target pitch is inside the measured mechanical range.

## Firmware Features

Main source file:

```text
src/Main.cpp
```

Implemented features:

- MPU6050 accelerometer and gyroscope calibration.
- Conversion of accelerometer readings to `m/s^2`.
- Conversion of gyroscope readings to `rad/s`.
- Fixed-period `200 Hz` control loop.
- 1D Kalman filter for pitch estimation.
- PID controller targeting `pi / 2 rad`.
- Signed 8-bit PWM output from `-255` to `+255`.
- Motor deadband handling: commands with magnitude at or below `90` are sent as
  zero.
- Bidirectional L298N motor control.
- Recovery lockout when the pitch is outside the estimated recoverable range.
- 3-second stable hold requirement before re-enabling control after lockout.
- 10 Hz serial telemetry for testing and tuning.

## Control Law

The controller computes:

```text
error = pitch - targetPitch
PWM = Kp * error + Ki * integral(error) + Kd * pitchRate
```

Current gains:

```cpp
float Kp = 700.0f;
float Ki = 0.0f;
float Kd = 55.0f;
```

The controller output is signed PWM, not torque. These gains therefore have PWM
units:

```text
Kp: PWM counts per radian
Kd: PWM counts per radian/second
Ki: PWM counts per radian-second
```

The output is saturated to the full 8-bit PWM range:

```text
-255 <= PWM <= +255
```

Hardware testing showed that the motor does not spin reliably unless PWM is
above `90` in either direction. The firmware therefore applies:

```text
if abs(PWM) <= 90, command = 0
```

Useful motor command ranges are:

```text
-255 to -91
0
+91 to +255
```

## Recovery Lockout

The firmware estimates the maximum recoverable pitch error using the static
gravity torque balance:

```text
motor torque = mass * gravity * COM distance * sin(angle error)
```

With the current motor rated torque:

```cpp
const float estimatedMaxMotorTorqueNm = 0.0686f;
```

and the measured body values:

```text
mass = 0.271 kg
COM distance = 0.0517 m
gravity = 9.81 m/s^2
```

the calculated maximum recoverable error is approximately:

```text
0.523 rad from pi / 2
```

If the pitch error exceeds this value, the firmware stops commanding the motor.
Once locked out, the pitch must be manually brought back within:

```text
plus or minus 0.3 rad from pi / 2
```

and held there for:

```text
3 seconds
```

before PID control resumes.

This behavior prevents the reaction wheel from continuously driving at full
power when the system is too far from the balance point to recover.

## Simulink Model

For a first-order presentation model, use the linearized pitch plant around
`pi / 2`:

```text
theta_error = pitch - pi / 2
```

The torque-to-angle plant is:

```text
           theta(s)              -1
G(s) = ------------- = -----------------------
          torque(s)     0.000375s^2 - 0.137445
```

Simulink Transfer Function block:

```text
Numerator:   [-1]
Denominator: [0.000375 0 -0.137445]
```

Because the firmware outputs PWM directly, a more firmware-matched model is:

```text
PID -> saturation [-255, 255] -> deadband -> Kpwm -> transfer function -> pitch error
```

where `Kpwm` is the effective torque per PWM count. It is not known yet because
the motor torque constant, winding resistance, driver voltage loss, and stall
torque have not been measured.

## Project Files

| Path | Purpose |
| --- | --- |
| `src/Main.cpp` | Main ESP32 firmware |
| `platformio.ini` | PlatformIO board, upload, monitor, and library settings |
| `NOTES.md` | Development notes, assumptions, and remaining measurements |
| `src/MotorDriver.txt` | Simple motor driver test sequence to find stall pwm |

## Build

This project uses PlatformIO with the ESP32 Arduino framework.

```powershell
C:\Users\Adins\.platformio\penv\Scripts\pio.exe run
```

Expected result:

```text
[SUCCESS]
```

## Upload

The configured upload port is `COM3`.

```powershell
C:\Users\Adins\.platformio\penv\Scripts\pio.exe run --target upload
```

## Serial Monitor

The configured monitor baud rate is `57600`.

```powershell
C:\Users\Adins\.platformio\penv\Scripts\pio.exe device monitor
```

Telemetry fields:

| Field | Meaning |
| --- | --- |
| `rawPitch` | Accelerometer-derived pitch angle |
| `kalmanPitch` | Filtered pitch estimate |
| `pitchRate` | Gyroscope pitch rate |
| `error` | Pitch error from `pi / 2` |
| `P`, `I`, `D` | PID term values |
| `pid` | Unsaturated PID calculation before integer command |
| `motor` | Final signed PWM command |
| `lockout` | `1` when recovery lockout is active |
| `holdMs` | Time inside the re-enable range during lockout |
| `maxErr` | Estimated maximum recoverable pitch error |

## Initial Test Procedure

1. Verify that ESP32, L298N, and bench supply grounds are common.
2. Keep motor power off and upload the firmware.
3. Open the serial monitor at `57600` baud.
4. Keep the IMU still during calibration.
5. Confirm the printed maximum recoverable pitch error.
6. Hold or physically constrain the rig before enabling motor power.
7. Enable motor power.
8. Tilt below `pi / 2` and confirm the motor command is negative.
9. Tilt above `pi / 2` and confirm the motor command is positive.
10. Move the rig outside the recoverable angle and confirm `lockout: 1`.
11. Move it back within `plus or minus 0.3 rad` of `pi / 2`.
12. Confirm `holdMs` counts to about `3000` before control resumes.

## Safety Notes

- Keep the rig physically constrained during early tests.
- The firmware can command full PWM from `-255` to `+255`, but commands with
  magnitude at or below `90` are sent as zero.
- Keep one hand near motor power cutoff during testing.
- Verify motor and pitch sign conventions before increasing gains.
- The recovery angle uses the motor's rated output torque, not measured stall
  torque or measured torque through the L298N driver.
- The L298N has significant voltage loss, so real motor torque may be lower
  than the estimate.

## Known Limitations

- Motor torque constant is not measured.
- Motor winding resistance is not measured.
- L298N voltage drop is not modeled.
- Pivot friction and wheel bearing friction are not modeled.
- The recovery angle is a static estimate and does not include body angular
  velocity or wheel speed saturation.
- Current gains are realistic starting values, but final values must be tuned
  on hardware.
