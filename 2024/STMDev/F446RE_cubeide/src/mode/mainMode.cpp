#include "mainMode.hpp"

void MainMode::before() {
    robot->chargeStart();
}

void MainMode::after() {
}

void MainMode::loop() {
    //     static int count = 0;
    //     count++;
    timer.reset();
    robot->getSensors(&robot->info);
    robot->rasSendSerial(robot->info, 8);
    robot->rasRecvSerial(); // sync

    if (robot->info.velX.vel == 0 && robot->info.velY.vel == 0 && robot->info.velAngler.vel == 0) {
        if (velZeroCount < 1000) {
            velZeroCount++;
        } else {
            velZeroCount = 1000;
        }
    } else {
        velZeroCount = 0;
    }

    if (!robot->info.status.emergencyStop && !robot->info.isUnderVoltage && velZeroCount < 1000) {
        robot->dribble(robot->info.dribblePower);

        // ストレートキックを優先する
        if (robot->info.kicker.straight > 0) {
            robot->kickStraight(robot->info.kicker.straight);
        } else if (robot->info.kicker.chip > 0) {
            robot->kickChip(robot->info.kicker.chip);
        }

        int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
        int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
        int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
        robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
    } else {
        // robot->motorDriver.setVelocityFF(0, 0, 0);
        robot->motorDriver.sendEmg();
        robot->dribble(0);
    }
    robot->led1 = robot->info.isHoldBall;
    printfDMA("Ball:%d Batt:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage);

    while (timer.read_us() < 1000)
        ; // 1ms time control
}