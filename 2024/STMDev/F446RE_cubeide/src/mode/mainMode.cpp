#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
    //     static int count = 0;
    //     count++;
    robot->getSensors(&robot->info);
    robot->rasSendSerial(robot->info, 8);
    robot->rasRecvSerial(); // sync

    robot->dribble(robot->info.dribblePower);

    // ストレートキックを優先する
    if (robot->info.kicker.straight > 0) {
        robot->kickStraight(robot->info.kicker.straight);
    } else if (robot->info.kicker.chip > 0) {
        robot->kickChip(robot->info.kicker.chip);
    }

    robot->motorDriver.setVelocityFF(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
    robot->led1 = robot->info.isHoldBall;

    printf("%d %d\n", robot->info.isHoldBall, robot->info.photoSensorValue);

    // Timer Timer;
    // Timer.reset();
    // for (int i = 0; i < 30; i++){
    //     robot->motorDriver.setVelocityFF(1000, 0, 0);
    //     HAL_Delay(100);
    // }
    // robot->motorDriver.setVelocityFF(0, 0, 0);
    // robot->motorDriver.setVelocityFF(0, 0, 0);

    // printfDMA("Bat:%d\n", robot->info.batteryVoltage);
    // printf("velX: %d, velY: %d, velAngler: %d\n", robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
}