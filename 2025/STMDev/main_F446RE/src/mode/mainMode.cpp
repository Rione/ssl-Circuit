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
      robot->uiRecvSerial(robot->info);       // UIからデータを受信する

      uiKickControl(robot->info);  // UIからの充電制御
      swKickControl(robot->info);  // スイッチからの充電制御

      robot->motorDriver.getVelocity(&robot->info.mdStatus.velX, &robot->info.mdStatus.velY, robot->info.mdStatus.motorAngularVelocity);  // モータードライバからロボットの速度を取得

      if (!robot->info.status.emergencyStop && robot->info.status.isSignalReceived) {
            // Robot is Running
            robot->sendDribble(robot->info.dribblePower);
            robot->sendKicker(robot->info);
            robot->kickerBoard.chargeControl(robot->info.status.doCharge);
            robot->sendMotor(robot->info, 5);  // 5msごとに送信
      } else {
            // Robot is Stop or Emergency Stop
            stopRobot(500);
            // printfDMA("Robot is Stop\n");
      }

      robot->led1 = robot->info.dribbleStatus.isDetectedBall;
      // printfDMA("Ball:%d Batt:%d Cap:%d doDirect:%d doDirectChip:%d directSt:%d directCh:%d Str:%d Chip:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage, robot->getcapChargeEstimate(), robot->info.status.doDirectKick, robot->info.status.doDirectChipKick, robot->info.kickerBoardDoDirectStatus.straight, robot->info.kickerBoardDoDirectStatus.chip, robot->info.kicker.straight, robot->info.kicker.chip);
      // printfDMA("Ball:%d, Photo:%d, NewDrib:%d, DribbleStatus:%d\n", robot->info.dribbleStatus.isHoldBall, robot->info.dribbleStatus.isDetectedBall, robot->info.dribbleStatus.isNewDrib, robot->info.dribbleStatus.data);
      while (timer.read_us() < 1000);  // 1ms time control
}