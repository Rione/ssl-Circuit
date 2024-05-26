#include "Kicker.hpp"

Kicker::Kicker(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t riseTime, uint16_t kickInterval)
    : kicker(htim, channel), _riseTime(riseTime), _kickInterval(kickInterval) {
    // riseTime [ms]
    // kickInterval [ms]
}

void Kicker::init() {
    kicker.init();
    available = true;
}

void Kicker::disable() {
    available = false;
}

void Kicker::enable() {
    available = true;
}

void Kicker::kick(float power) {
    if (!available) return;
    if (_intervalTimer.read_ms() < _kickInterval) return;
    power = Constrain(power, 0.0, 1.0);
    kicker.write(power);
    _riseTimer.reset();
    _intervalTimer.reset();
}

void Kicker::update() {
    if (_riseTimer.read_ms() > _riseTime) {
        kicker.write(0.0);
    }
}

void Kicker::disCharge() {
    printf("start discharging\n");
    for (size_t i = 0; i < 300; i++) {
        kicker.write(0.7);
        HAL_Delay(30);
        kicker.write(0.0);
        HAL_Delay(10);
        printf("...");
    }
}