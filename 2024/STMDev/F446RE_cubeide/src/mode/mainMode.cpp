#include "mainMode.hpp"

void MainMode::before() {
    // robot->chargeStart();
    robot->led0 = 1;    
    timer.reset();
}

void MainMode::after() {
    robot->led0 = 0;
}

void MainMode::loop() {
    //     static int count = 0;
    //     count++;
    // timer.reset();
    // if (!robot->info.status.emergencyStop || !robot->info.isUnderVoltage) {
    //     robot->getSensors(&robot->info);
    //     robot->rasSendSerial(robot->info, 8);
    //     robot->rasRecvSerial(); // sync

    //     robot->dribble(robot->info.dribblePower);

    //     // ストレートキックを優先する
    //     if (robot->info.kicker.straight > 0) {
    //         robot->kickStraight(robot->info.kicker.straight);
    //     } else if (robot->info.kicker.chip > 0) {
    //         robot->kickChip(robot->info.kicker.chip);
    //     }

    //     int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
    //     int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
    //     int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
    //     robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
    // } else {
    //     // robot->motorDriver.setVelocityFF(0, 0, 0);
    //     robot->motorDriver.sendEmg();
    //     robot->dribble(0);
    // }
    // robot->led1 = robot->info.isHoldBall;
    // printfDMA("Ball:%d Batt:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage);

    // while (timer.read_us() < 1000) ; // 1ms time control

    if(timer.read_ms() > 1500){
        timer.reset();
        robot->led0 = !robot->led0;
    }  
    
}