#include "mainMode.hpp"

#include "ModePolicy.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
  robot->getSensors(&robot->info);
  robot->bnoGet(robot->info, IMU_UPDATE_INTERVAL_MS);  // BNO055からセンサデータを取得
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

  const mode_policy::Drive drive = mode_policy::arbitrate(BENCH_MODES != 0, isTestingMotor || isTestingDribbler,
                                                          robot->info.status.emergencyStop,
                                                          robot->info.status.isSignalReceived);
  if (drive == mode_policy::Drive::kUiTest) {
    // === UIテスト中（台上用のビルドだけ。非常停止と信号より先に動く） ===
    if (isTestingMotor) {
      robot->motorDriver.setVelocityFF(100, 0, 0); // モーターテスト: 100 mm/s 前進
    } else {
      robot->motorDriver.setVelocityFF(0, 0, 0);
    }
    
    if (isTestingDribbler) {
      robot->sendDribble(100); // テスト時パワー100 (新旧ドリブラー仕様に準拠)
    } else {
      robot->sendDribble(0);
    }
    
    robot->sendKicker(robot->info);
  } else if (drive == mode_policy::Drive::kFollowPi) {
    // === 通常の試合中 (RasPiからの信号あり) ===
    robot->sendDribble(robot->info.dribblePower);
    robot->sendKicker(robot->info);
    // UI/スイッチ操作後15秒間はPiの充放電指令を無視する
    if (robot->manageByUserCounter.read_ms() >= 15000) {
      robot->kickerBoard.chargeControl(robot->info.status.doCharge ? CHARGE : DISCHARGE);
    }
    robot->sendMotor(robot->info, 10);
  } else {
    // === 非常停止・待機状態 ===
    stopRobot(500);
  }

  robot->led1 = robot->info.dribbleStatus.isDetectedBall;
  waitNextPeriod(1000);  // 1ms周期
}