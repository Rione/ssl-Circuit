#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
    static Timer doDirectToggle;
    // robot->info.status.isSignalReceived = true;

    // robot->info.kicker.straight = 0;
    // robot->info.status.doDirectKick = false;

    // robot->info.velX.vel = 0;
    // robot->info.velY.vel = 0;

    // if (doDirectToggle.read_ms() > 5000) {
    //     doDirectToggle.reset();
    //     robot->info.status.doDirectChipKick = !robot->info.status.doDirectChipKick;
    //     if (robot->info.status.doDirectChipKick) {
    //         robot->info.kicker.chip = 100;
    //     } else {
    //         robot->info.kicker.chip = 0;
    //     }
    // }

    timer.reset();
    robot->getSensors(&robot->info);
    robot->rasSendSerial(robot->info, 8);
    robot->rasRecvSerial();  // sync
    robot->checkRobotRest(); // ロボットが停止しているか確認
    boosterManager();        // 昇圧回路の管理
    if (!robot->info.status.emergencyStop && robot->info.status.isSignalReceived) {
        // Robot is Running
        robot->dribble(robot->info.dribblePower);
        robot->processKicker();
        int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
        int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
        int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
        robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
    } else {
        // Robot is Stop or Emergency Stop
        robot->stopRobot(500);
    }
    robot->led1 = robot->info.isHoldBall;
    printfDMA("Ball:%d Batt:%d Cap:%d doDirect:%d doDirectChip:%d directSt:%d directCh:%d Str:%d Chip:%d togle:%ld\n", robot->info.photoSensorValue, robot->info.batteryVoltage, robot->getCapChargeCertitude(), robot->info.status.doDirectKick, robot->info.status.doDirectChipKick, robot->info.kickerBoardDoDirectStatus.straight, robot->info.kickerBoardDoDirectStatus.chip, robot->info.kicker.straight, robot->info.kicker.chip, doDirectToggle.read_ms());
    while (timer.read_us() < 1000)
        ; // 1ms time control
}