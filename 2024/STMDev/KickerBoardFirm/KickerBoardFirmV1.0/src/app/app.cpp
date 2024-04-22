#include "app.h"
#include "DigitalInOut.hpp"

DigitalOut led0(LED0_GPIO_Port, LED0_Pin);

void setup() {
}

void main_app() {
    setup();
    while (1) {
        led0 = !led0;
        HAL_Delay(500);
    }
}