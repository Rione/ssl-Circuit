#include "local_controller.hpp"

#include <cmath>

LocalController::LocalController() {}

void LocalController::Stop(Robot& robot) {
  robot.omni_drive.SetFree();

  robot.kicker.CancelDirect(Kicker::kStraight);
  robot.kicker.CancelDirect(Kicker::kChip);
  robot.SendKicker(robot.info);

  robot.SendDribble(0);
}
