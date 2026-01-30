#include "main_mode.hpp"

void MainMode::loop() {
      timer.reset();

      // シリアル通信
      robot->RockRecvSerial(robot->info);
      robot->RockSendSerial(robot->info);
      robot->UiRecvSerial(robot->info);
      robot->UiSendSerial(robot->info);

      while (timer.read_us() < 1000);
}