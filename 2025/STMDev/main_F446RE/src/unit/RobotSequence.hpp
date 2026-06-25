#ifndef __RobotSequence__
#define __RobotSequence__

#include "main.h"
#include "config.h"

#include "Robot.hpp"

void stopRobot(uint16_t interval);
void uiKickControl(RobotInfo_t &info);
void buzzerControl(RobotInfo_t &info);
void swKickControl(RobotInfo_t &info);
void ballMoving();
void dribbleHoldBack();

#endif