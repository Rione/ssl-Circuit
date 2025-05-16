#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
    timer.reset();
    robot->getSensors(&robot->info);
    robot->rasSendSerial(robot->info, 8);
    robot->rasRecvSerial();  // sync
    robot->checkRobotRest(robot->info); // ロボットが停止しているか確認

    // doDirectが入っている時はブザーを鳴らすように変更した
    if (robot->info.status.doDirectKick || robot->info.status.doDirectChipKick) {
        robot->UIrobotInfo.buzzer = playType::SUCCESS;
    } else {
        robot->UIrobotInfo.buzzer = playType::NONE;
    }

    boosterManager(); // 昇圧回路の管理  //UIとの通信
    if (!robot->info.status.emergencyStop && robot->info.status.isSignalReceived) {
        // Robot is Running
        robot->dribble(robot->info.dribblePower);
        // robot->processKicker(robot->info, robot->kickerBoard);

        // processKicker
        if (robot->info.kicker.straight > 0) {
            robot->kickerBoard.kick(STRAIGHT, robot->info.kicker.straight, robot->info.status.doDirectKick);
        } else if (robot->info.kicker.chip > 0) {
            robot->kickerBoard.kick(CHIP, robot->info.kicker.chip, robot->info.status.doDirectChipKick);
        } else {
            // どっちも0の場合はキックしない
            if (robot->info.status.doDirectKick != robot->info.kickerBoardDoDirectStatus.straight && robot->info.kickerBoardDoDirectStatus.straight) {
                // kickStraight(0, false); // パワー0のキックを投げてdoDirectをリセットする
                robot->kickerBoard.resetDoDirect(STRAIGHT);
            }
            if (robot->info.status.doDirectChipKick != robot->info.kickerBoardDoDirectStatus.chip && robot->info.kickerBoardDoDirectStatus.chip) {
                // kickChip(0, false); // パワー0のキックを投げてdoDirectをリセットする
                robot->kickerBoard.resetDoDirect(CHIP);
                
            }
        }
        
        robot->kickerBoard.chargeControl(robot->info.status.doCharge);

        int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
        int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
        int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
        static uint8_t md_sendcount = 0;
        md_sendcount ++;
        if (md_sendcount % 5 == 0) {
            robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
        }
    } else {
        // Robot is Stop or Emergency Stop
        robot->stopRobot(*robot, 500);
        // printfDMA("Robot is Stop\n");
    }
    robot->led1 = robot->info.dribbleStatus.isDetectedBall;
    // printfDMA("Ball:%d Batt:%d Cap:%d doDirect:%d doDirectChip:%d directSt:%d directCh:%d Str:%d Chip:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage, robot->getcapChargeEstimate(), robot->info.status.doDirectKick, robot->info.status.doDirectChipKick, robot->info.kickerBoardDoDirectStatus.straight, robot->info.kickerBoardDoDirectStatus.chip, robot->info.kicker.straight, robot->info.kicker.chip);
    printfDMA("Ball:%d, Photo:%d, NewDrib:%d, DribbleStatus:%d\n", robot->info.dribbleStatus.isHoldBall, robot->info.dribbleStatus.isDetectedBall, robot->info.dribbleStatus.isNewDrib, robot->info.dribbleStatus.data);
    while (timer.read_us() < 1000)
        ; // 1ms time control
}