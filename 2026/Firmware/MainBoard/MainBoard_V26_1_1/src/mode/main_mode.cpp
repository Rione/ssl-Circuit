#include "main_mode.hpp"

MainMode::MainMode(Robot* robot) : Mode(robot) {}

void MainMode::Loop() {
  timer_.reset();

  // シリアル通信
  robot_->RockRecvSerial(robot_->info);
  robot_->RockSendSerial(robot_->info);

  robot_->omni_drive.SetVel(robot_->info.vel_x.vel, robot_->info.vel_y.vel, robot_->info.vel_angular.vel);
  robot_->omni_drive.Recv();

  while (timer_.read_us() < robot_params::kControlLoopDt * 1000);
}