#include "ReactionWheelController.h"

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include <math.h>

namespace ReactionWheel {
namespace {

MPU6050 mpu;

/*
  Reaction wheel balancing controller.

  The firmware estimates pitch with an MPU6050, runs a fixed-rate PID
  controller around the upright pitch target, and commands a bidirectional
  brushed DC motor through an L298N driver. The PID output is a signed 8-bit
  PWM command, not a torque command.
*/

// L298N motor driver pins.
const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

// ESP32 LEDC PWM configuration for the motor enable pin.
const int pwmChannel = 0;
const int frequency = 1000;
const int resolution = 8;
const int maxPwm = 255;
const int minActivePwm = 90;

// Control-loop and telemetry timing.
const unsigned long controlPeriodUs = 5000; // 200 Hz
const unsigned long debugPeriodMs = 100;    // 10 Hz

// Measured mounted pitch range. The target balance angle is centered by test.
const float pitchLeftLimit = 0.566f;
const float pitchRightLimit = 2.58f;
const float targetPitch = 1.150f;

// Physical parameters for simulation/model-based tuning.
const float wheelMassKg = 0.046f;
const float wheelRadiusM = 0.160f;
const float comDistanceToPivotM = 0.0517f;
const float wheelMomentOfInertiaKgM2 = 0.5f * wheelMassKg * wheelRadiusM * wheelRadiusM;
const float bodyMomentOfInertiaKgM2 = 0.000375f;
const float totalBodyMassWithWheelKg = 0.271f;
const float motorVoltageV = 12.0f;
const float motorMaxEfficiencyRpm = 10386.0f;
const float gravityMps2 = 9.81f;
const float gramCmToNm = 0.0000980665f;

// Motor torque model from datasheet values:
// - max efficiency: 1.55 A, 96 g.cm
// - stall: 10 A, 714 g.cm
// The controller uses the current-limited stall-line estimate because the
// motor driver will limit current before the motor reaches stall torque.
const float motorMaxEfficiencyCurrentA = 1.55f;
const float motorMaxEfficiencyTorqueGcm = 96.0f;
const float motorStallTorqueGcm = 714.0f;
const float motorStallTorqueNm = motorStallTorqueGcm * gramCmToNm;
const float motorStallCurrentA = 10.0f;
const float motorCurrentLimitA = 2.0f;
const float estimatedMaxMotorTorqueNm = motorStallTorqueNm * (motorCurrentLimitA / motorStallCurrentA);
const float maxAllowedPitchErrorRad = 10.0f;
const float recoveryReenableErrorRad = 1.0f;
const unsigned long recoveryReenableHoldMs = 3000;

// Controller behavior near upright.
const float balanceDeadbandRad = 0.015f;
const float rateDeadbandRps = 0.08f;
const int maxCommandStepPerCycle = 32;

/*
  Sign conventions:
  - Pitch increases as the wheel/body tilts right.
  - Positive motor command = clockwise wheel spin:
    IN1 HIGH, IN2 LOW, sends the wheel/body right.
  - Negative motor command = counterclockwise wheel spin:
    IN1 LOW, IN2 HIGH, sends the wheel/body left.

  If the IMU or motor wiring changes, adjust these signs before changing the
  control law. This keeps the controller math readable and hardware-independent.
*/
const float angleSign = 1.0f;
const float gyroPitchSign = 1.0f;
const int motorSign = 1;

// Full-range PWM gains. The units are PWM counts per radian and PWM counts per
// radian/second because the controller output drives PWM directly.
float Kp = 5300.0f;
float Ki = 30.0f;
float Kd = 650.0f;

// Kalman tuning.
float kalmanQAngle = 0.001f;
float kalmanQBias = 0.003f;
float kalmanRMeasure = 0.03f;

// Sensor bias estimates.
float Ax_bias = 0.0f;
float Ay_bias = 0.0f;
float Az_bias = 0.0f;
float Gx_bias = 0.0f;
float Gy_bias = 0.0f;
float Gz_bias = 0.0f;

// Latest converted IMU readings.
float Ax_mps2 = 0.0f;
float Ay_mps2 = 0.0f;
float Az_mps2 = 0.0f;
float Gx_rps = 0.0f;
float Gy_rps = 0.0f;
float Gz_rps = 0.0f;

float rawPitch = 0.0f;
float roll = 0.0f;
float kalmanPitch = targetPitch;
float gyroBias = 0.0f;
float kalmanP[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

float pidIntegral = 0.0f;
float lastError = 0.0f;
float lastP = 0.0f;
float lastI = 0.0f;
float lastD = 0.0f;
float lastPidOutput = 0.0f;
int lastMotorCommand = 0;
int rampedMotorCommand = 0;
float maxRecoverableErrorRad = 0.0f;
float estimatedRecoverableErrorRad = 0.0f;
bool recoveryLockedOut = true;
unsigned long recoveryWithinRangeSinceMs = 0;

unsigned long nextControlTimeUs = 0;
unsigned long lastDebugMs = 0;

} // namespace

/**
 * Read one raw MPU6050 sample and convert it into physical units.
 */
void readIMU() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  Ax_mps2 = (ax / 16384.0f) * gravityMps2;
  Ay_mps2 = (ay / 16384.0f) * gravityMps2;
  Az_mps2 = (az / 16384.0f) * gravityMps2;

  Gx_rps = (gx / 131.0f) * PI / 180.0f;
  Gy_rps = (gy / 131.0f) * PI / 180.0f;
  Gz_rps = (gz / 131.0f) * PI / 180.0f;
}

/**
 * Estimate static accelerometer and gyroscope offsets.
 */
void calibrateIMU() {
  const int samples = 2000;

  float Ax_sum = 0.0f;
  float Ay_sum = 0.0f;
  float Az_sum = 0.0f;
  float Gx_sum = 0.0f;
  float Gy_sum = 0.0f;
  float Gz_sum = 0.0f;

  Serial.println("Keep IMU still while calibrating...");
  delay(500);

  for (int i = 0; i < samples; i++) {
    readIMU();

    Ax_sum += Ax_mps2;
    Ay_sum += Ay_mps2;
    Az_sum += Az_mps2;

    Gx_sum += Gx_rps;
    Gy_sum += Gy_rps;
    Gz_sum += Gz_rps;

    delay(5);
  }

  Ax_bias = Ax_sum / samples;
  Ay_bias = Ay_sum / samples;
  Az_bias = Az_sum / samples - gravityMps2;

  Gx_bias = Gx_sum / samples;
  Gy_bias = Gy_sum / samples;
  Gz_bias = Gz_sum / samples;

  Serial.println("Calibration done.");
  delay(1000);
}

/**
 * Normalize an angle to the range [-pi, pi].
 */
float wrapToPi(float angle) {
  while (angle > PI) {
    angle -= 2.0f * PI;
  }
  while (angle < -PI) {
    angle += 2.0f * PI;
  }
  return angle;
}

/**
 * Calculate the moment of inertia for a solid disk reaction wheel.
 */
float calculateWheelMomentOfInertiaKgM2(float massKg, float radiusM) {
  return 0.5f * massKg * radiusM * radiusM;
}

/**
 * Estimate the largest pitch error where motor torque can still overcome
 * gravity torque about the pivot.
 */
float calculateMaxRecoverableErrorRad() {
  float gravityTorqueScale = totalBodyMassWithWheelKg * gravityMps2 * comDistanceToPivotM;
  if (gravityTorqueScale <= 0.0f) {
    return maxAllowedPitchErrorRad;
  }

  float torqueRatio = estimatedMaxMotorTorqueNm / gravityTorqueScale;
  torqueRatio = constrain(torqueRatio, 0.0f, 1.0f);
  return asinf(torqueRatio);
}

/**
 * Check whether the filtered pitch is inside the measured mounted range.
 */
bool isPitchInsideMountedRange(float pitch) {
  return pitch >= pitchLeftLimit && pitch <= pitchRightLimit;
}

/**
 * Update the 1D Kalman filter for pitch estimation.
 */
float updateKalman(float measuredAngle, float measuredRate, float dt) {
  float rate = measuredRate - gyroBias;
  kalmanPitch += dt * rate;

  kalmanP[0][0] += dt * (dt * kalmanP[1][1] - kalmanP[0][1] - kalmanP[1][0] + kalmanQAngle);
  kalmanP[0][1] -= dt * kalmanP[1][1];
  kalmanP[1][0] -= dt * kalmanP[1][1];
  kalmanP[1][1] += kalmanQBias * dt;

  float innovation = wrapToPi(measuredAngle - kalmanPitch);
  float S = kalmanP[0][0] + kalmanRMeasure;
  float K0 = kalmanP[0][0] / S;
  float K1 = kalmanP[1][0] / S;

  kalmanPitch += K0 * innovation;
  gyroBias += K1 * innovation;

  float P00 = kalmanP[0][0];
  float P01 = kalmanP[0][1];

  kalmanP[0][0] -= K0 * P00;
  kalmanP[0][1] -= K0 * P01;
  kalmanP[1][0] -= K1 * P00;
  kalmanP[1][1] -= K1 * P01;

  return kalmanPitch;
}

/**
 * Command the L298N motor driver from a signed PWM request.
 */
void setMotorCommand(int command) {
  command *= motorSign;

  if (command == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, 0);
    lastMotorCommand = 0;
    return;
  }

  int pwm = abs(command);
  pwm = constrain(pwm, minActivePwm, maxPwm);

  if (command > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  ledcWrite(pwmChannel, pwm);
  lastMotorCommand = command > 0 ? pwm : -pwm;
}

/**
 * Slew-limit the signed PWM request before applying it to the motor.
 */
int rampMotorCommand(int targetCommand) {
  targetCommand = constrain(targetCommand, -maxPwm, maxPwm);

  int delta = targetCommand - rampedMotorCommand;
  delta = constrain(delta, -maxCommandStepPerCycle, maxCommandStepPerCycle);
  rampedMotorCommand += delta;

  return rampedMotorCommand;
}

/**
 * Compute the PID motor command from the filtered pitch angle.
 */
int computePid(float angle, float pitchRate, float dt) {
  float error = wrapToPi(angle - targetPitch);

  if (fabsf(error) < balanceDeadbandRad && fabsf(pitchRate) < rateDeadbandRps) {
    pidIntegral = 0.0f;
    lastError = error;
    lastP = 0.0f;
    lastI = 0.0f;
    lastD = 0.0f;
    lastPidOutput = 0.0f;
    return 0;
  }

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -1.0f, 1.0f);

  float derivative = pitchRate;
  lastError = error;

  lastP = Kp * error;
  lastI = Ki * pidIntegral;
  lastD = Kd * derivative;
  lastPidOutput = lastP + lastI + lastD;

  int command = (int)roundf(lastPidOutput);
  command = constrain(command, -maxPwm, maxPwm);

  return command;
}

/**
 * Initialize serial output, motor PWM, I2C, the MPU6050, and filter state.
 */
void controllerSetup() {
  Serial.begin(57600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  ledcSetup(pwmChannel, frequency, resolution);
  ledcAttachPin(ENA, pwmChannel);
  setMotorCommand(0);

  Wire.begin(21, 22);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU connected.");
  } else {
    Serial.println("MPU failed to connect.");
  }

  calibrateIMU();

  readIMU();
  float Ax_final = Ax_mps2 - Ax_bias;
  float Ay_final = Ay_mps2 - Ay_bias;
  float Az_final = Az_mps2 - Az_bias;
  rawPitch = angleSign * atan2(Ay_final, Az_final);
  kalmanPitch = rawPitch;
  lastError = wrapToPi(kalmanPitch - targetPitch);
  estimatedRecoverableErrorRad = calculateMaxRecoverableErrorRad();
  maxRecoverableErrorRad = maxAllowedPitchErrorRad;

  Serial.print("Reaction wheel radius: ");
  Serial.print(wheelRadiusM, 3);
  Serial.println(" m");
  Serial.print("Reaction wheel inertia estimate: ");
  Serial.print(wheelMomentOfInertiaKgM2, 7);
  Serial.println(" kg*m^2");
  Serial.print("Calculated recoverable pitch error estimate: ");
  Serial.print(estimatedRecoverableErrorRad, 4);
  Serial.println(" rad");
  Serial.print("Configured max pitch error: ");
  Serial.print(maxRecoverableErrorRad, 4);
  Serial.println(" rad");
  Serial.println("Hold near upright to arm controller.");

  nextControlTimeUs = micros();
}

/**
 * Main real-time control loop.
 */
void controllerLoop() {
  unsigned long nowUs = micros();
  if ((long)(nowUs - nextControlTimeUs) < 0) {
    return;
  }
  nextControlTimeUs += controlPeriodUs;

  const float dt = controlPeriodUs / 1000000.0f;

  readIMU();

  float Ax_final = Ax_mps2 - Ax_bias;
  float Ay_final = Ay_mps2 - Ay_bias;
  float Az_final = Az_mps2 - Az_bias;
  float Gx_final = Gx_rps - Gx_bias;

  roll = atan2(-Ax_final, sqrt(Ay_final * Ay_final + Az_final * Az_final));
  rawPitch = angleSign * atan2(Ay_final, Az_final);
  float pitchRate = gyroPitchSign * Gx_final;

  float filteredPitch = updateKalman(rawPitch, pitchRate, dt);
  float pitchError = wrapToPi(filteredPitch - targetPitch);
  bool pitchInsideMountedRange = isPitchInsideMountedRange(filteredPitch);
  unsigned long nowMs = millis();

  if (recoveryLockedOut) {
    if (pitchInsideMountedRange && fabsf(pitchError) <= recoveryReenableErrorRad) {
      if (recoveryWithinRangeSinceMs == 0) {
        recoveryWithinRangeSinceMs = nowMs;
      } else if (nowMs - recoveryWithinRangeSinceMs >= recoveryReenableHoldMs) {
        recoveryLockedOut = false;
        recoveryWithinRangeSinceMs = 0;
        pidIntegral = 0.0f;
        lastError = pitchError;
      }
    } else {
      recoveryWithinRangeSinceMs = 0;
    }
  } else if (!pitchInsideMountedRange || fabsf(pitchError) > maxRecoverableErrorRad) {
    recoveryLockedOut = true;
    recoveryWithinRangeSinceMs = 0;
    pidIntegral = 0.0f;
  }

  int motorCommand = 0;
  if (recoveryLockedOut) {
    rampedMotorCommand = 0;
    lastError = pitchError;
    lastP = 0.0f;
    lastI = 0.0f;
    lastD = 0.0f;
    lastPidOutput = 0.0f;
  } else {
    motorCommand = rampMotorCommand(computePid(filteredPitch, pitchRate, dt));
  }
  setMotorCommand(motorCommand);

  if (nowMs - lastDebugMs >= debugPeriodMs) {
    lastDebugMs = nowMs;

    Serial.print("rawPitch: ");
    Serial.print(rawPitch, 4);
    Serial.print(" kalmanPitch: ");
    Serial.print(filteredPitch, 4);
    Serial.print(" pitchRate: ");
    Serial.print(pitchRate, 4);
    Serial.print(" error: ");
    Serial.print(lastError, 4);
    Serial.print(" P: ");
    Serial.print(lastP, 2);
    Serial.print(" I: ");
    Serial.print(lastI, 2);
    Serial.print(" D: ");
    Serial.print(lastD, 2);
    Serial.print(" pid: ");
    Serial.print(lastPidOutput, 2);
    Serial.print(" motor: ");
    Serial.print(lastMotorCommand);
    Serial.print(" lockout: ");
    Serial.print(recoveryLockedOut ? 1 : 0);
    Serial.print(" holdMs: ");
    if (recoveryLockedOut && recoveryWithinRangeSinceMs != 0) {
      Serial.print(nowMs - recoveryWithinRangeSinceMs);
    } else {
      Serial.print(0);
    }
    Serial.print(" maxErr: ");
    Serial.println(maxRecoverableErrorRad, 4);
  }
}

} // namespace ReactionWheel
