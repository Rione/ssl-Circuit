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
      robot->uiRecvSerial(robot->info);    // UIからデータを受信
      robot->checkRobotRest(robot->info);  // ロボットが停止しているか確認

      buzzerControl(robot->info);  // ブザーの制御

      robot->uiSendSerial(robot->info, 100);  // UIにデータを送信する
      uiKickControl(robot->info);             // UIからのキック指示を実行

      robot->motorDriver.getVelocity(&robot->info.mdStatus.velX, &robot->info.mdStatus.velY, robot->info.mdStatus.motorAngularVelocity);  // モータードライバからロボットの速度を取得

      bool isEStop = robot->info.status.emergencyStop || robot->info.uiStatus.emergencyStop;
      bool isTestingMotor = robot->info.isMotorTesting && (robot->info.motorTestTimer.read_ms() < 3000);
      bool isTestingDribbler = robot->info.isDribblerTesting;

      // モーターテストが3秒経過したらフラグを自動でOFFにする
      if (robot->info.isMotorTesting && robot->info.motorTestTimer.read_ms() >= 3000) {
            robot->info.isMotorTesting = false;
      }

      if (isEStop) {
            // E-Stop時は全停止＋放電
            stopRobot(500);
            if (robot->info.uiStatus.emergencyStop) { // UIからのE-Stopの場合のみ安全策として放電追加
                robot->kickerBoard.kick(STRAIGHT, 20);
                robot->kickerBoard.chargeControl(DISCHARGE);
                robot->info.uiStatus.emergencyStop = 0; // 1回だけ実行するようにリセットするか、UI側で離されたら0になる
            }
      } else if (isTestingMotor || isTestingDribbler) {
            // テスト中の場合、安全装置をバイパス
            if (isTestingMotor) {
                  robot->motorDriver.setVelocityFF(100, 0, 0); // 100 mm/s 前進
            } else {
                  robot->motorDriver.setVelocityFF(0, 0, 0);
            }
            if (isTestingDribbler) {
                  robot->sendDribble(100); // 最大パワー
            } else {
                  robot->sendDribble(0);
            }
            robot->sendKicker(robot->info);
            robot->kickerBoard.chargeControl(robot->info.status.doCharge);
      } else if (robot->info.status.isSignalReceived) {
            // Robot is Running
            robot->sendDribble(robot->info.dribblePower);
            robot->sendKicker(robot->info);
            robot->kickerBoard.chargeControl(robot->info.status.doCharge);
            robot->sendMotor(robot->info, 5);  // 5msごとに送信
      } else {
            // Robot is Stop
            stopRobot(500);
            // printfDMA("Robot is Stop\n");
      }

      robot->led1 = robot->info.dribbleStatus.isDetectedBall;
      while (timer.read_us() < 1000);  // 1ms time control
}