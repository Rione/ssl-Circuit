#include "local_controller.h"

#include <stdio.h>

void LocalController_Init(LocalController* self) {
  (void)self;
}

void LocalController_Stop(LocalController* self, Robot* robot) {
  (void)self;
  OmniDrive_SetFree(&robot->omni_drive);

  Kicker_CancelDirect(&robot->kicker);
  Kicker_CancelDirect(&robot->kicker);
  Robot_SendKicker(robot, &robot->info);

  Robot_SendDribble(robot, 0, 0);
}
