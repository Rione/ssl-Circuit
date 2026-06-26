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

  robot->uiRecvSerial(robot->info);    // UIからデータ受信
  uiKickControl(robot->info);          // UIからのキック・チャージ制御
  swKickControl(robot->info);          // 物理スイッチからのキック・チャージ制御

  buzzerControl(robot->info);  // ブザーの制御

  robot->uiSendSerial(robot->info, 100);  // UIにデータを送信する

  robot->motorDriver.getVelocity(&robot->info.mdStatus.velX, &robot->info.mdStatus.velY, robot->info.mdStatus.motorAngularVelocity);  // モータードライバからロボットの速度を取得

  bool isTestingMotor = robot->info.isMotorTesting && (robot->info.motorTestTimer.read_ms() < 3000);
  bool isTestingDribbler = robot->info.isDribblerTesting;

  if (robot->info.isMotorTesting && robot->info.motorTestTimer.read_ms() >= 3000) {
    robot->info.isMotorTesting = false;
  }

  if (robot->info.status.emergencyStop) {
    stopRobot(500);
  } else if (isTestingMotor || isTestingDribbler || robot->manageByUserCounter.read_ms() < 15000) {
    // === UIテスト中 ===
    // Raspberry Piからの充放電指令(doCharge)を上書きしないようにする！
    if (isTestingMotor) {
      robot->motorDriver.setVelocityFF(500, 0, 0); // モーターテスト: 500 mm/s 前進
    } else {
      robot->motorDriver.setVelocityFF(0, 0, 0);
    }
    
    if (isTestingDribbler) {
      robot->sendDribble(100); // ドリブラーテスト
    } else {
      robot->sendDribble(0);
    }
    
    robot->sendKicker(robot->info);
  } else if (robot->info.status.isSignalReceived) {
    // === 通常の試合中 (RasPiからの信号あり) ===
    robot->sendDribble(robot->info.dribblePower);
    robot->sendKicker(robot->info);
    robot->sendMotor(robot->info, 10);
  } else {
    // === 待機状態 ===
    stopRobot(500);
  }

  robot->led1 = robot->info.dribbleStatus.isDetectedBall;
  while (timer.read_us() < 1000);  // 1ms time control
}