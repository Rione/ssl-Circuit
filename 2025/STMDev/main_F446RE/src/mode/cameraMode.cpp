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

      robot->kickerBoard.chargeControl(1);
      // printf("Camera X: %d, Y: %d\n", robot->info.camera.x, robot->info.camera.y);
      printf("aa%d\n", robot->info.dribbleStatus.isDetectedBall);
      int16_t vel_x, vel_y;
      const uint16_t max_speed = 500;
      if (robot->info.camera.x == 0 && robot->info.camera.y == 0) {
            robot->motorDriver.setVelocityFF(0, 0, 2000);
      } else if (robot->info.dribbleStatus.isHoldBall == true) {
            robot->kickerBoard.kick(0, 50, 0);

      } else {
            vel_x = Constrain(robot->info.camera.y * 30, -max_speed, max_speed);
            vel_y = 0;
            robot->motorDriver.setVelocityFF(vel_x, vel_y, (127 - robot->info.camera.x) * 20);
      }
      if (robot->info.dribbleStatus.isDetectedBall == true || (robot->info.camera.y < 5 && robot->info.camera.y != 0)) {
            robot->sendDribble(50);
      } else {
            robot->sendDribble(0);
      }

      robot->led1 = robot->info.dribbleStatus.isDetectedBall;
      while (timer.read_us() < 1000);  // 1ms time control
}