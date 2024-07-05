#include "Kicker.hpp"
#include "DMAStream.h"

Kicker::Kicker(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t riseTime, uint16_t kickInterval)
    : kicker(htim, channel), _riseTime(riseTime), _kickInterval(kickInterval), available(false), isDischarging(false) {
    // riseTime [ms]
    // kickInterval [ms]
}

void Kicker::init() {
    kicker.init();
    relatedKicker = nullptr;
}

void Kicker::disable() {
    available = false;
}

void Kicker::enable() {
    available = true;
}

void Kicker::addRelatedKicker(Kicker *kicker) {
    relatedKicker = kicker;
}

void Kicker::kick(float power) {
    if (power == 0.0) return;
    if (!available || isDischarging) return;
    if (_intervalTimer.read_ms() < _kickInterval) return;
    if (relatedKicker != nullptr) relatedKicker->disable();
    power = Constrain(power, 0.0, 1.0);
    kicker.write(power);
    _riseTimer.reset();
    _intervalTimer.reset();
}

void Kicker::update() {
    if (isDischarging) return;
    if (_riseTimer.read_ms() > _riseTime) {
        kicker.write(0.0);
    }
    if (_riseTimer.read_ms() > 300) {
        if (relatedKicker != nullptr) relatedKicker->enable();
    }
}

void Kicker::setDirectKick(bool state, float power) {
    doDirectKick = state;
    directKickPower = power;
}

float Kicker::getDirectKickPower() {
    return directKickPower;
}

bool Kicker::getDoDirecStatus() {
    return doDirectKick;
}

bool Kicker::directKick(bool holdBallState) {
    if (holdBallState && doDirectKick) {
        kick(directKickPower);
        return true;
    }
    return false;
}

void Kicker::disCharge() {
    printf("start discharging\n");
    isDischarging = true;
    for (size_t i = 0; i < 1300; i++) {
        kicker.write(0.3);
        HAL_Delay(2);
        kicker.write(0.0);
        HAL_Delay(2);
    }
    isDischarging = false;
}