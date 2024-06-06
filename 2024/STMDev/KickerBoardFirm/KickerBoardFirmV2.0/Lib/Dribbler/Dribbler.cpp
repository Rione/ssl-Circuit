#include "Dribbler.hpp"

Dribbler::Dribbler(TIM_HandleTypeDef *htim, uint32_t channel) : pin(htim, channel) {
}

void Dribbler::init() {
    pin.init();
    HAL_Delay(100);
    pin.write(0.1);
    HAL_Delay(100);
    pin.write(0.05);
    HAL_Delay(100);
}

void Dribbler::run() {
    pin.write(0.1);
}

void Dribbler::stop() {
    pin.write(0.05);
}

void Dribbler::write(float power) {
    // power = 0.0 ~ 1.0
    power = Constrain(power, 0.0, 1.0);
    pin.write(power * 0.05 + 0.05); // 0.05 ~ 0.1
}