#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
  timer.reset();
  robot->getSensors(&robot->info);
  // robot->bnoGet(robot->info);  // BNO055からセンサデータを取得
  robot->rasSendSerial(robot->info, 8);
  robot->rasRecvSerial(robot->info);   // sync
  robot->checkRobotRest(robot->info);  // ロボットが停止しているか確認

  buzzerControl(robot->info);  // ブザーの制御

  robot->uiSendSerial(robot->info, 100);  // UIにデータを送信する

  robot->motorDriver.getVelocity(&robot->info.mdStatus.velX, &robot->info.mdStatus.velY, robot->info.mdStatus.motorAngularVelocity);  // モータードライバからロボットの速度を取得

  if (!robot->info.status.emergencyStop && robot->info.status.isSignalReceived) {
    // Robot is Running
    robot->sendDribble(robot->info.dribblePower);
    robot->sendKicker(robot->info);
    robot->kickerBoard.chargeControl(robot->info.status.doCharge);
    robot->sendMotor(robot->info, 10);  // 10msごとに送信
  } else {
    // Robot is Stop or Emergency Stop
    stopRobot(500);
    // printfDMA("Robot is Stop\n");
  }

  robot->led1 = robot->info.dribbleStatus.isDetectedBall;
  while (timer.read_us() < 1000);  // 1ms time control
}