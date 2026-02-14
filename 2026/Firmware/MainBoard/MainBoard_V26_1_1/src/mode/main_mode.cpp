#include "main_mode.hpp"

MainMode::MainMode(Robot* robot) : Mode(robot) {}

void MainMode::Loop() {
  static Timer timer;
  timer.reset();

  // シリアル通信
  robot_->RockRecvSerial(robot_->info);

  // UIからの操作を更新
  robot_->UpdateFromUi();

  robot_->RockSendSerial(robot_->info);
  robot_->omni_drive.Recv();

  if (!robot_->info.status.emergency_stop && robot_->info.status.is_signal_received) {
    // Robot is Running
    robot_->SendDribble(robot_->info.dribble_power);
    robot_->SendKicker(robot_->info);

    if (robot_->info.status.do_charge) {
      robot_->kicker.Charge();
    } else {
      robot_->kicker.Discharge();
    }
    robot_->SendOmniDrive(robot_->info, 5);  // 5msごとに送信
  } else {
    // Robot is Stop or Emergency Stop
  }

  while (timer.read_ms() < robot_params::kControlLoopDt);
}