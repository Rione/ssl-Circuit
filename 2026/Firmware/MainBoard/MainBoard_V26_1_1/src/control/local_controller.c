#include "local_controller.h"

#include <stdio.h>

void LocalController_Init(LocalController* self) {
  (void)self;
}

void LocalController_Stop(LocalController* self, Robot* robot) {
  (void)self;
  OmniDrive_SetFree(&robot->omni_drive);
  // OmniDrive_SetVel(&robot->omni_drive, 200, 0, 0);

  Kicker_CancelDirect(&robot->kicker, KICKER_STRAIGHT);
  Kicker_CancelDirect(&robot->kicker, KICKER_CHIP);
  Robot_SendKicker(robot, &robot->info);

  Robot_SendDribble(robot, 0, 0);
}
