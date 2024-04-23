#include "app.h"
#include "DigitalInOut.hpp"

int data = 0;
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);

void setup() {
}

void main_app() {
    setup();
    while (1) {
        led0 = !led0;
        data++;
        HAL_Delay(500);
    }
}