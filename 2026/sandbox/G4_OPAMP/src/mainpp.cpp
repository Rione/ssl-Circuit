#include "mainpp.hpp"
#include "AnalogIn.hpp"

AnalogIn amp_sense(&hadc2, ADC_CHANNEL_3);

void Setup(void) {
}

void MainLoop() {
    while (1) {
        printf("%d\n", amp_sense.read());
        HAL_Delay(3);
    }
}