#include "main_mode.hpp"

void MainMode::Loop() {
      timer.reset();

      // シリアル通信
      robot->RockRecvSerial(robot->info);
      robot->RockSendSerial(robot->info);
      robot->UiRecvSerial(robot->info);
      robot->UiSendSerial(robot->info);

      while (timer.read_us() < params::CONTROL_LOOP_DT * 1000);
}