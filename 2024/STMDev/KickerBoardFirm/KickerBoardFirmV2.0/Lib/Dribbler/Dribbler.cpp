#include "Dribbler.hpp"

Dribbler::Dribbler(TIM_HandleTypeDef *htim, uint32_t channel) : pin(htim, channel) {
}

void Dribbler::init() {
    pin.init();
    HAL_Delay(100);
    pin.write(0.1);
    HAL_Delay(1000);
    pin.write(0.05);
    HAL_Delay(1000);
}

void Dribbler::run() {
    pin.write(0.08);
}

void Dribbler::stop() {
    pin.write(0.025);
}

void Dribbler::write(float power) {
}