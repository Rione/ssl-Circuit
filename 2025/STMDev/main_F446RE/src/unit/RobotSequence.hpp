#ifndef __ RobotSequence__
#define __ RobotSequence__

#include "main.h"
#include "config.h"

#include "Robot.hpp"

void stopRobot(uint16_t interval);
void sendKicker(RobotInfo_t &info);
void sendMotor(RobotInfo_t &info, uint8_t interval);
void uiKickControl(RobotInfo_t &info);
void buzzerControl(RobotInfo_t &info);
void swKickControl(RobotInfo_t &info);

#endif