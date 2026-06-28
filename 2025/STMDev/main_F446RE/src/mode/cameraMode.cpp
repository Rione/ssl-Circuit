#include "cameraMode.hpp"

void CameraMode::before() {
}

void CameraMode::after() {
}

Timer camera_correction_timer;

void CameraMode::loop() {
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

      int16_t vel_x, vel_y;
      const uint16_t max_speed = 300;

      static uint16_t camera_x, camera_y;
      if (robot->info.camera.x == 0 && robot->info.camera.y == 0) {
            if (camera_correction_timer.read() > 2) {
                  camera_x = 0;
                  camera_y = 0;
            }
      } else {
            camera_x = robot->info.camera.x;
            camera_y = robot->info.camera.y;
            camera_correction_timer.reset();
      }

      bool isTestingMotor = robot->info.isMotorTesting && (robot->info.motorTestTimer.read_ms() < 3000);
      bool isTestingDribbler = robot->info.isDribblerTesting;

      if (robot->info.isMotorTesting && robot->info.motorTestTimer.read_ms() >= 3000) {
            robot->info.isMotorTesting = false;
      }

      if (isTestingMotor || isTestingDribbler) {
            // === UIテスト中（2026の仕様に合わせ、安全裁定をバイパス） ===
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
      } else if (robot->info.status.emergencyStop) {
            stopRobot(500);
      } else if (robot->info.status.isSignalReceived) {
            // === 試合中 (RasPi信号あり) ===
            if (camera_x == 0 && camera_y == 0) {
                  robot->motorDriver.setVelocityFF(0, 0, 1500);
            } else {
                  vel_x = Constrain(robot->info.camera.y * 30, -max_speed, max_speed);
                  vel_y = 0;
                  robot->motorDriver.setVelocityFF(vel_x, vel_y, (127 - robot->info.camera.x) * 30);
            }
            robot->sendDribble(robot->info.dribblePower);
            robot->sendKicker(robot->info);
            robot->kickerBoard.chargeControl(robot->info.status.doCharge ? CHARGE : DISCHARGE);
            robot->sendMotor(robot->info, 10);
      } else {
            // === 待機状態 ===
            stopRobot(500);
      }
      // if (robot->info.dribbleStatus.isDetectedBall == true || (robot->info.camera.y < 5 && robot->info.camera.y != 0)) {
      //       robot->sendDribble(50);
      // } else {
      //       robot->sendDribble(0);
      // }

      robot->led1 = robot->info.dribbleStatus.isDetectedBall;
      while (timer.read_us() < 1000);  // 1ms time control
}