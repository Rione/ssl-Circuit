#include "testMode.hpp"

#include "MotorDriver.hpp"
#include "Robot.hpp"
#include "RobotSequence.hpp"
#include "arduino_compat/arduino_compat.h"
extern Robot robot;

TestMode::TestMode(char letter, const char name[], Robot* robotPtr)
    : Mode(letter, name, robotPtr), estimator() {
  estimator.init(0.5f, 0.1f);  // Initial parameters
}

void TestMode::before() {}

void TestMode::loop() {}

void TestMode::after() {}
