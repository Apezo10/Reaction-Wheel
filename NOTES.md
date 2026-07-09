# Reaction Wheel Project Notes

## Project Goal

Build a 1-dimensional reaction wheel balancing system using an ESP32, MPU6050 IMU, L298N motor driver, and bidirectional brushed DC motor.

The controller should:

- Balance the body about the pitch axis.
- Hold the desired balance angle at the configured `targetPitch`.
- Reject disturbances.
- Use a Kalman filter for pitch estimation.
- Use PID control for motor torque command.

## Current Project Location

```text
C:\Users\Adins\OneDrive\Documents\VScode Projects\Reaction Wheel
```

Firmware source files:

```text
include\ReactionWheelController.h
src\Main.cpp
src\ReactionWheelController.cpp
```

## Current PlatformIO Setup

Board/environment:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

Serial/upload:

```ini
upload_port = COM3
monitor_port = COM3
upload_speed = 57600
monitor_speed = 57600
```

Current library dependency:

```ini
lib_deps = electroniccats/MPU6050
```

## Known Hardware

Microcontroller:

- ESP32 dev board.
- Powered through USB.

IMU:

- MPU6050.
- IMU will be facing upward.
- Current code uses I2C on ESP32 pins:
  - SDA: GPIO 21
  - SCL: GPIO 22

Motor driver:

- L298N.
- Powered by a 12 V bench supply.
- Must share common ground with the ESP32.

Motor:

- Brushed DC motor.
- Bidirectional.
- Driven through the L298N.

## Current Motor Wiring / Pin Assumptions

Current `src\ReactionWheelController.cpp` pin definitions:

```cpp
const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;
```

PWM setup:

```cpp
const int pwmChannel = 0;
const int frequency = 1000;
const int resolution = 8;
```

Current motor test result:

- Motor appears to stall around PWM `88`.
- Minimum useful PWM is `90` in either direction.
- Nonzero controller commands below PWM `90` are raised to PWM `90` to overcome stiction.
- PWM command range is 8-bit: `0` to `255`.

Measured motor direction with motor label at bottom:

- `IN1 HIGH`, `IN2 LOW`: wheel spins clockwise and sends the wheel/body right.
- `IN1 LOW`, `IN2 HIGH`: wheel spins counterclockwise and sends the wheel/body left.
- Therefore:
  - `pitch < targetPitch`: spin counterclockwise.
  - `pitch > targetPitch`: spin clockwise.

## Current IMU Code Behavior

Current code:

- Initializes the MPU6050.
- Calibrates accelerometer and gyro bias using 2000 samples.
- Converts accelerometer readings to `m/s^2`.
- Converts gyro readings to `rad/s`.
- Calculates pitch and roll from accelerometer data.

Current pitch/roll formulas:

```cpp
roll = atan2(-Ax_final, sqrt(Ay_final * Ay_final + Az_final * Az_final));
pitch = atan2(Ay_final, Az_final);
```

Important:

- Pitch when tilted all the way left: `0.566 rad`.
- Pitch when tilted all the way right: `2.58 rad`.
- Target pitch: `1.150 rad`.
- Pitch increases as the wheel/body tilts right.
- The mounted pitch range brackets `targetPitch`, so the current pitch convention can be used for initial closed-loop testing.

## Intended Embedded Control Structure

Recommended next control implementation:

- Fixed-timestep loop, likely `200 Hz`. Complete in `src\ReactionWheelController.cpp`.
- Loop period: `5 ms`. Complete in `src\ReactionWheelController.cpp`.
- Kalman filter combining. Complete in `src\ReactionWheelController.cpp`.
  - accelerometer-derived pitch angle
  - gyro pitch rate
- PID controller using filtered pitch.
- Target angle:

```cpp
const float targetPitch = 1.150f;
```

- Bidirectional L298N motor command function:
  - positive command: one motor direction
  - negative command: opposite motor direction
  - zero command: motor off or brake/coast depending on chosen behavior
- Deadband compensation:
  - if command is nonzero, require minimum active PWM `90`
  - clamp output to `255`
- Serial debug output:
  - raw pitch
  - Kalman pitch
  - gyro rate
  - PID error
  - PID terms
  - motor command/PWM

## PID / Sign Tuning Notes

PID gains are not known yet.

User may create a Simulink simulation to estimate initial PID gains.

The embedded code should expose easy constants for:

```cpp
float Kp;
float Ki;
float Kd;
```

The code should also expose sign constants because real motor/IMU orientation may be reversed:

```cpp
const float angleSign = 1.0;   // or -1.0
const int motorSign = 1;       // or -1
```

Initial sign convention implemented in `src\ReactionWheelController.cpp`:

- Error is calculated as `pitch - targetPitch`.
- Positive command spins the wheel clockwise (`IN1 HIGH`, `IN2 LOW`).
- Negative command spins the wheel counterclockwise (`IN1 LOW`, `IN2 HIGH`).
- This matches the measured requirement:
  - `pitch < targetPitch` produces negative/CCW command.
  - `pitch > targetPitch` produces positive/CW command.

These signs should still be verified gently before aggressive PID tuning.

## Information Still Needed

### Required for Safe Embedded Control

1. Confirm the real mounted IMU pitch reading at the desired balance position. Complete.
   - Left limit: `0.566 rad`.
   - Right limit: `2.58 rad`.
   - Target: `1.150 rad`.

2. Confirm positive pitch direction. Complete.
   - Pitch increases as the wheel/body tilts right.

3. Confirm motor correction direction. Complete.
   - For `pitch > targetPitch`, command clockwise (`IN1 HIGH`, `IN2 LOW`).
   - For `pitch < targetPitch`, command counterclockwise (`IN1 LOW`, `IN2 HIGH`).

4. Confirm L298N wiring.
   - ESP32 GPIO 25 to L298N ENA.
   - ESP32 GPIO 26 to L298N IN1.
   - ESP32 GPIO 27 to L298N IN2.
   - ESP32 GND connected to L298N GND.
   - Bench supply ground connected to L298N ground.

5. Measure motor deadband in both directions. Initial value complete.
   - Minimum active PWM: `90`.
   - Nonzero commands below `90` are raised to `90`.

6. Choose motor behavior at zero command.
   - Coast: both direction pins low or PWM zero.
   - Brake: both direction pins same state depending on L298N behavior.

### Useful for Simulink / Model-Based PID Design

Mass/inertia values are not strictly required to write the embedded PID/Kalman code, but they are useful for a realistic simulation.

Collect if possible:

- Total body/pendulum mass: `0.271 kg` with wheel.
- Distance from pivot to body center of mass: `0.0517 m`.
- Body moment of inertia about pitch axis: `0.000375 kg*m^2`.
- Reaction wheel mass: `0.046 kg`.
- Reaction wheel radius: `0.160 m`.
- Reaction wheel moment of inertia: `0.0005888 kg*m^2`.
- Motor torque constant, if known.
- Motor speed constant, if known.
- Motor winding resistance, if known.
- Motor no-load speed at 12 V: `550 rpm`.
- Maximum wheel RPM, if known: `550 rpm` no-load from motor spec.
- Gear ratio, if any.
- Pivot friction estimate, if any.
- Wheel bearing friction estimate, if any.
- Motor driver voltage drop estimate, if known.

### Useful Safety / Practical Details

- Does the physical rig have hard stops?
- Maximum safe wheel speed.
- Maximum safe PWM during early testing.
- Whether the rig can be held by hand during first closed-loop tests.
- Whether there is an emergency stop button or easy power cutoff.

## Suggested Next Step

The current motor step-test logic in `src\ReactionWheelController.cpp` has been replaced with a control-ready structure:

1. Keep IMU calibration. Complete.
2. Add fixed `5 ms` control loop timing. Complete.
3. Add 1D Kalman filter for pitch. Complete.
4. Add PID controller targeting `targetPitch`. Complete.
5. Add L298N bidirectional motor command function. Complete.
6. Add conservative output limits for first test. Complete.
7. Print debug values at a slower rate, such as `10 Hz`, while the control loop runs at `200 Hz`. Complete.

Next hardware step:

1. Upload and test with the rig held securely.
2. Confirm that below-target pitch produces a negative motor debug value and counterclockwise spin.
3. Confirm that above-target pitch produces a positive motor debug value and clockwise spin.
4. Tune `Kp`, `Ki`, `Kd`, motor command limits, and Kalman constants cautiously.
