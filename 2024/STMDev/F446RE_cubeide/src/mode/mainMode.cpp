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
        robot->dribble(0);
        robot->motorDriver.sendEmg();
    }

    { // ロボットの状態に関わらず常に行う処理
        if (robot->swImu.isRelease()) {
            if (robot->swImu.readPressedTime() > 1600) {
                robot->discharge();
                robot->led2 = false;
                printf("discharge start\n");
            } else if (robot->swImu.readPressedTime() > 800) {
                robot->chargeStart();
                robot->led2 = true;
                printf("charge start\n");
            } else if (robot->swImu.readPressedTime() > 200) {
                robot->kickStraight(100);
                printf("kick\n");
            }
        }
        robot->led1 = robot->info.isHoldBall;
        printfDMA("Ball:%d Batt:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage);
    }

    while (timer.read_us() < 1000)
        ; // 1ms time control
}