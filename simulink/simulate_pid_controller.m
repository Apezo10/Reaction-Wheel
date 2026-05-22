function result = simulate_pid_controller()
%SIMULATE_PID_CONTROLLER MATLAB-only smoke test for the reaction wheel PID.
%
% This uses the same constants and command shaping as the generated Simulink
% model. It is useful when you want quick numeric feedback without opening the
% Simulink diagram.

Ts = 0.005;
tEnd = 5.0;
t = 0:Ts:tEnd;

Kp = 700.0;
Ki = 0.0;
Kd = 55.0;
maxPwm = 255;
minActivePwm = 90;
maxCommandStepPerCycle = 12;
balanceDeadbandRad = 0.015;
rateDeadbandRps = 0.08;

J = 0.000375;
m = 0.271;
g = 9.81;
l = 0.0517;
motorStallTorqueGcm = 714.0;
gramCmToNm = 0.0000980665;
motorStallCurrentA = 10.0;
motorCurrentLimitA = 2.0;
maxMotorTorque = motorStallTorqueGcm * gramCmToNm * ...
    (motorCurrentLimitA / motorStallCurrentA);
torquePerPwm = maxMotorTorque / maxPwm;

pitchError = zeros(size(t));
pitchRate = zeros(size(t));
pidOutput = zeros(size(t));
motorPwm = zeros(size(t));
motorTorque = zeros(size(t));

pitchError(1) = 0.08;
pitchRate(1) = 0.0;
integralError = 0.0;
rampedCommand = 0.0;

for k = 1:numel(t)-1
    if abs(pitchError(k)) < balanceDeadbandRad && abs(pitchRate(k)) < rateDeadbandRps
        integralError = 0.0;
        targetCommand = 0.0;
        rampedCommand = 0.0;
    else
        integralError = integralError + pitchError(k) * Ts;
        integralError = min(max(integralError, -1.0), 1.0);
        pidOutput(k) = Kp * pitchError(k) + Ki * integralError + Kd * pitchRate(k);
        targetCommand = round(min(max(pidOutput(k), -maxPwm), maxPwm));
        delta = min(max(targetCommand - rampedCommand, -maxCommandStepPerCycle), maxCommandStepPerCycle);
        rampedCommand = rampedCommand + delta;
    end

    if rampedCommand == 0
        motorPwm(k) = 0;
    elseif rampedCommand > 0
        motorPwm(k) = min(max(rampedCommand, minActivePwm), maxPwm);
    else
        motorPwm(k) = -min(max(abs(rampedCommand), minActivePwm), maxPwm);
    end

    motorTorque(k) = torquePerPwm * motorPwm(k);
    pitchAccel = (m * g * l / J) * sin(pitchError(k)) - motorTorque(k) / J;
    pitchRate(k + 1) = pitchRate(k) + Ts * pitchAccel;
    pitchError(k + 1) = pitchError(k) + Ts * pitchRate(k + 1);
end

result = table(t(:), pitchError(:), pitchRate(:), pidOutput(:), motorPwm(:), motorTorque(:), ...
    "VariableNames", ["time_s", "pitch_error_rad", "pitch_rate_rad_s", "pid_output", "motor_pwm", "motor_torque_Nm"]);

fprintf("Final pitch error: %.5f rad\n", pitchError(end));
fprintf("Max abs pitch error: %.5f rad\n", max(abs(pitchError)));
fprintf("Peak PWM: %.0f\n", max(abs(motorPwm)));
end
