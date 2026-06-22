#pragma once

namespace ReactionWheel {

void controllerSetup();
void controllerLoop();

void readIMU();
void calibrateIMU();
float wrapToPi(float angle);
float calculateWheelMomentOfInertiaKgM2(float massKg, float radiusM);
float calculateMaxRecoverableErrorRad();
bool isPitchInsideMountedRange(float pitch);
float updateKalman(float measuredAngle, float measuredRate, float dt);
void setMotorCommand(int command);
int rampMotorCommand(int targetCommand);
int computePid(float angle, float pitchRate, float dt);

} // namespace ReactionWheel
