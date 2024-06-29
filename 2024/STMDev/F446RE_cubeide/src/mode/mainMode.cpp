#include "mainMode.hpp"

void MainMode::before() {
    robot->chargeStart();
}

void MainMode::after() {
}

void MainMode::loop() {
    timer.reset();
    robot->getSensors(&robot->info);
    robot->rasSendSerial(robot->info, 8);
    robot->rasRecvSerial(); // sync
    checkRobotStop();       // ロボットが停止しているか確認
    boosterManager();       // 昇圧回路の管理2
    if (!robot->info.status.emergencyStop && !isRobotStop && robot->info.status.isSignalReceived) {
        // Robot is Running
        robot->dribble(robot->info.dribblePower);
        // ストレートキックを優先する
        if (robot->info.kicker.straight > 0) {
            if (robot->info.status.doDirectKick) {
                if (robot->info.isHoldBall)
                    robot->kickStraight(robot->info.kicker.straight);
            } else {
                robot->kickStraight(robot->info.kicker.straight);
            }
        } else if (robot->info.kicker.chip > 0) {
            if (robot->info.status.doDirectChipKick) {
                if (robot->info.isHoldBall)
                    robot->kickChip(robot->info.kicker.chip);
            } else {
                robot->kickChip(robot->info.kicker.chip);
            }
        }

        int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
        int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
        int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
        robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
    } else {
        // Robot is Stop or Emergency Stop
        robot->stopRobot(500);
    }
    robot->led1 = robot->info.isHoldBall;
    printfDMA("Ball:%d Batt:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage);
    while (timer.read_us() < 1000)
        ; // 1ms time control
}