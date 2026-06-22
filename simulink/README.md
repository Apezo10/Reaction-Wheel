# Reaction Wheel PID Validation Model

This folder contains MATLAB/Simulink tooling to validate the PID controller in
`src/ReactionWheelController.cpp`.

## Files

- `build_reaction_wheel_pid_validation_model.m` creates
  `reaction_wheel_pid_validation.slx` and runs a smoke simulation.
- `simulate_pid_controller.m` runs the same controller/plant approximation in
  plain MATLAB and returns a result table.

## Model Assumptions

- State is pitch error around upright: `pitch - targetPitch`.
- The plant is the nonlinear inverted-pendulum pitch approximation:
  `theta_ddot = (m*g*l/J)*sin(theta) - motorTorque/J`.
- The controller mirrors firmware behavior:
- `Kp = 5300`
- `Ki = 30`
- `Kd = 650`
  - `Ts = 0.005 s`
  - PWM saturation at `+/-255`
  - command slew limit of `12` PWM counts per control cycle
  - balance deadband of `0.015 rad`
  - rate deadband of `0.08 rad/s`
  - nonzero motor commands raised to minimum active PWM `90`

The motor torque mapping is still an estimate based on the current-limited
stall torque line already used in the firmware. Replace `torquePerPwm` with a
measured value when you have motor/driver data.

## Usage

From MATLAB in this folder:

```matlab
build_reaction_wheel_pid_validation_model
```

For a quick MATLAB-only check:

```matlab
result = simulate_pid_controller;
```
