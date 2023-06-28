#ifndef _SETUP_H_
#define _SETUP_H_

#include "mbed.h"
#include "pinDefs.h"
#include "setup_common.h"
#include "RIMode.h"
#include "RIHardwareAPI.h"
#include "attitude.h"
// mode management
char mode;               // mode letter
int8_t runningModeIndex; // mode index
int8_t runningModeIndexPrev;
RIMode target;
RIMode targetPrev;
Timer timer;
extern RobotInfo info;
#endif