#include "main_mode.hpp"

void MainMode::Loop() {
      timer.reset();

      // シリアル通信
      robot->RockRecvSerial(robot->info);
      robot->RockSendSerial(robot->info);
      robot->UiRecvSerial(robot->info);
      robot->UiSendSerial(robot->info);

      robot->motorDrive.SetVel(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
      robot->motorDrive.Recv();

      while (timer.read_us() < params::CONTROL_LOOP_DT * 1000);
}