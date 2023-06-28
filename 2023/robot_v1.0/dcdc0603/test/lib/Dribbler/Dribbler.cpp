#include "mbed.h"
#include "Dribbler.h"

Dribbler::Dribbler(PinName dribblerPin) : motor(dribblerPin) {
    setPower(1.0);
    turnMaxInterval = 15; // 加速のためのインターバル[ms]
    timer.start();
    offTimer.start();
}

void Dribbler::setPower(float p) {
    targetPowerPrev = targetPower;
    targetPower = (p < 0.0 ? 0.0 : (p > 1.0 ? 1.0 : p));
    if (targetPower < targetPowerPrev) offTimer.reset();
}

void Dribbler::dribble() {
    // if (targetPower >= 0.0) {
    //     if (power < targetPower) {
    //         if (timer.read_ms() > turnMaxInterval) {
    //             power += 0.02;
    //             timer.reset();
    //         }
    //     } else {
    //         if (offTimer.read_ms() > 600) power = targetPower;
    //     }
    // } else {
    //     targetPower = 0;
    //     power = 0;
    // }
    // motor.write(power);
    motor.write(targetPower);
}

void Dribbler::turnOff() {
    motor.write(0);
    power = 0;
}

float Dribbler::getPower() {
    return power;
}