#include "app.h"
#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "adc.h"
#include "usart.h"
#include <cstdio>
#include <cstring>
#include "PWMSingleN.hpp"
#include "CAN.hpp"
#include "DMAStream.h"
#include "Timer.hpp"

CANBus can(&hcan, 0x100);
DigitalOut debugled(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
DigitalOut canled(CAN_LED_GPIO_Port, CAN_LED_Pin);
DigitalOut charge(CHARGE_GPIO_Port, CHARGE_Pin);
DigitalIn sw1(SW1_GPIO_Port, SW1_Pin);
DigitalIn sw2(SW2_GPIO_Port, SW2_Pin);
PwmSingleOut straightkicker(&htim15, TIM_CHANNEL_2);
PwmSingleOut chipkicker(&htim3, TIM_CHANNEL_2);
PwmSingleOut chip(&htim15, TIM_CHANNEL_2);
PwmSingleOutN dribbler(&htim1, TIM_CHANNEL_2);
Timer timer;
Timer timer2;
Timer statusSenderTimer;
Timer sw1Timer;
Timer dischargeTime;

bool chargeFlag = false;
bool straightFlag = false;
bool chipFlag = false;
bool dribbleFlag = false;
bool dribbleStopFlag = false;

bool sw1Pressed = false;

CANBus::CANData canRecvData = {
    .stdId = 0x01,
    .data = {0, 0, 0, 0, 0, 0, 0, 0},
};

CANBus::CANData canPhotoData = {
    .stdId = 0x03,
    .data = {0, 0, 0, 0, 0, 0, 0, 0},
};

void setup() {
    straightkicker.init();
    chipkicker.init();
    dribbler.init();
    can.init();
    dribbleinit();
}

void processData(int data) {
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
        switch (canRecvData.stdId) {
        case 0x10:
            chargeFlag = true;
            break;
        case 0x11:
            straightFlag = true;
            break;
        case 0x12:
            chipFlag = true;
            break;
        case 0x13:
            dribbleFlag = true;
            break;
        case 0x14:
            dribbleStopFlag = true;
            break;
        default:
            break;
        }
    }
    canled = !canled;
}

void main_app() {
    setup();
    // printf("starttt\n\r");
    charge = 0;
    readADC();
    timer.reset();
    timer2.reset();
    statusSenderTimer.reset();
    while (1) {
        uint8_t adcValue = readADC();
        // printfDMA("adcValue: %d\n", adcValue);
        if (statusSenderTimer.read_ms() > 10) {
            // 10msに1回送信
            canPhotoData.stdId = 0x123;
            canPhotoData.data[0] = adcValue;
            can.send(canPhotoData);
            statusSenderTimer.reset();
        }
        //  updatePhotoDetection(adcValue);
        // switch (canRecvData.stdId) {
        // case 0x10: // 16
        //     chargeDevice();
        //     break;
        // case 0x11: // 17
        //     straightkick();
        //     break;
        // case 0x12: // 18
        //     chipkick();
        //     break;
        // case 0x13: // 19
        //     dribble();
        //     break;
        // case 0x14: // 20
        //     dribblestop();
        //     break;
        // }
        if (chargeFlag) {
            chargeDevice();
            chargeFlag = false;
        }
        if (straightFlag) {
            straightkick();
            straightFlag = false;
        }
        if (chipFlag) {
            chipkick();
            chipFlag = false;
        }
        if (dribbleFlag) {
            dribble();
            dribbleFlag = false;
        }
        if (dribbleStopFlag) {
            dribblestop();
            dribbleStopFlag = false;
        }

        if (sw1.read() == 0) {
            if (sw1Pressed == false) {
                sw1Timer.reset();
                sw1Pressed = true;
            }
        } else {
            if (sw1Pressed) {
                if (sw1Timer.read_ms() > 1000) {
                    discharge();
                    dischargeTime.reset();
                } else {
                    if (dischargeTime.read_ms() > 1000) {
                        chargeFlag = true;
                    }
                }
                sw1Pressed = false;
            }
        }
        if (sw2.read() == 0) {
            straightFlag = true;
        }
        printfDMA("adcValue: %d %d %d\n", adcValue, sw1.read(), sw2.read());
        HAL_Delay(10);
    }
}

void chargeDevice() {
    if (timer.read_ms() > 1000) {
        charge = 1;
        printfDMA("charge start\n");
        HAL_Delay(3000);
        charge = 0;
        timer.reset();
    }
}

void discharge() {
    printfDMA("discharge start\n");
    for (size_t i = 0; i < 100; i++) {
        straightkicker.write(0.2);
        HAL_Delay(15);
        straightkicker.write(0.0);
        HAL_Delay(15);
    }
    printfDMA("discharge end\n");
}

int readADC() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    return HAL_ADC_GetValue(&hadc1);
}

int photo(int adc_Value) {
    return adc_Value <= 100;
}

void updatePhotoDetection(int adcValue) {
    if (photo(adcValue)) {
        canPhotoData.data[2] = 0x06;
        can.send(canPhotoData);
    } else {
        canPhotoData.data[2] = 0x07;
        can.send(canPhotoData);
    }
}

void straightkick() {
    if (timer2.read_ms() > 3000) {
        straightkicker.write(0.3);
        debugled = 1;
        printfDMA("straight\n");
        HAL_Delay(100);
        straightkicker.write(0);
        debugled = 0;
        timer2.reset();
    }
}

void chipkick() {
    if (timer2.read_ms() > 3000) {
        chipkicker.write(1);
        debugled = 1;
        printfDMA("chipkick\n");
        HAL_Delay(100);
        chipkicker.write(0);
        debugled = 0;
        timer2.reset();
    }
}

void dribble() {
    dribbler.write(0.08);
}

void dribbleinit() {
    dribbler.write(0.1);
    debugled = !debugled;
    HAL_Delay(1000);
    dribbler.write(0.05);
    debugled = !debugled;
    HAL_Delay(1000);
}

void dribblestop() {
    dribbler.write(0.025);
    printfDMA("Dribblestop\n");
}

void deactivateAll() {
    charge = 0;
    straightkicker.write(0);
    chipkicker.write(0);
    dribbler.write(0);
}
