#include <Arduino.h>
#include "ReactionWheelController.h"

void setup() {
  ReactionWheel::controllerSetup();
}

void loop() {
  ReactionWheel::controllerLoop();
}
