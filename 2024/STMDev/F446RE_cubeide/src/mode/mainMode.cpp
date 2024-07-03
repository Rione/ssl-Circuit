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
    printfDMA("Ball:%d Batt:%d Cap:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage, robot->getCapChargeCertitude());
    while (timer.read_us() < 1000)
        ; // 1ms time control
}