#include "mainMode.hpp"

void MainMode::before() {
}

void MainMode::after() {
}

void MainMode::loop() {
    //     static int count = 0;
    //     count++;
    robot->rasSendSerial(robot->info);
    robot->rasRecvSerial();
    robot->motorDriver.setVelocityFF(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
    
    //     // velAngulerがどうしても16bitで送られてくるので、どうしたらいいのかわからない。取り合えず0.001をかけている。
    //     printf("velX: %d, velY: %d, velAngler: %d\n", robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
}