#ifndef _MODES_
#define _MODES_
// modes
#include "modeTest.h"
#include "modeMain.h"
#include "modeTestIMU.h"
#include "modeDrib.h"
#include "modeKick.h"
#include "modeKick_force.h"
// mode registration
const int8_t modeIndex = 6;
const RIMode modes[modeIndex] = {modeMain, modeTest, modeTestIMU, modeDrib,
                                 modeKick, modeKickForce};

#endif