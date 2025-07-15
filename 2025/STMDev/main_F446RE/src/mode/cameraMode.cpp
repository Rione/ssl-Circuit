#include "cameraMode.hpp"

void CameraMode::before() {
}

void CameraMode::after() {
}

void CameraMode::loop() {
      timer.reset();
      robot->getSensors(&robot->info);
      // robot->bnoGet(robot->info);  // BNO055からセンサデータを取得
      robot->rasSendSerial(robot->info, 8);
      robot->rasRecvSerial(robot->info);   // sync
      robot->checkRobotRest(robot->info);  // ロボットが停止しているか確認

      buzzerControl(robot->info);  // ブザーの制御

      robot->uiSendSerial(robot->info, 100);  // UIにデータを送信する

      robot->motorDriver.getVelocity(&robot->info.mdStatus.velX, &robot->info.mdStatus.velY, robot->info.mdStatus.motorAngularVelocity);  // モータードライバからロボットの速度を取得

      printf("Camera X: %d, Y: %d\n", robot->info.camera.x, robot->info.camera.y);

      if (robot->info.camera.x == 0 && robot->info.camera.y == 0) {
            stopRobot(500);
      } else {
            robot->motorDriver.setVelocityFF((robot->info.camera.y - 75) * 2.5, (65 - robot->info.camera.x) * 3, 0);
      }

      robot->led1 = robot->info.dribbleStatus.isDetectedBall;
      while (timer.read_us() < 1000);  // 1ms time control
}