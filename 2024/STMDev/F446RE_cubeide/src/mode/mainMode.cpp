#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
    //     static int count = 0;
    //     count++;
    robot->getSensors(robot->info);
    robot->rasSendSerial(robot->info, 10);
    robot->rasRecvSerial();
    robot->motorDriver.setVelocityFF(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
    printfDMA("Bat:%d\n", robot->info.batteryVoltage);
    //     printf("velX: %d, velY: %d, velAngler: %d\n", robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
}