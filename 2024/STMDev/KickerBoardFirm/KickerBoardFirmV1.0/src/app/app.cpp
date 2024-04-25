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
PwmSingleOut straightkicker(&htim15, TIM_CHANNEL_2);
PwmSingleOut chipkicker(&htim3, TIM_CHANNEL_2);
PwmSingleOutN dribbler(&htim1, TIM_CHANNEL_2);
Timer timer;

CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {0, 0, 0, 0, 0, 0, 0, 8},
};

CANBus::CANData canPhotoData = {
    .stdId = 0x555,
    .data = {0, 0, 0, 0, 0, 0, 0, 8},
};

void setup() {
    straightkicker.init();
    chipkicker.init();
    dribbler.init();
    can.init();
}

// typedef enum {
//     IDLE,
//     STRAIGHT_KICK_START,
//     STRAIGHT_KICK_END,
//     CHIP_KICK_START,
//     DRIBBLE_START,
//     DRIBBLE_MID,
//     DRIBBLE_END
// } KickState;

// volatile int kickState = canRecvData.data[2];
// volatile uint32_t tickCount = 0;

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// if (htim->Instance == &htim6) {
//     tickCount++;

//     switch (kickState) {
//     case 0x01:
//         if (tickCount >= 20) { // 200ms後
//             straightkicker.write(0);
//             debugled = 1;
//             tickCount = 0;
//         }
//         break;
//     case 0x02:
//         if (tickCount >= 100) { // 1000ms後
//             chipkicker.write(0.5);
//             tickCount = 0;
//         }
//         break;
//     case 0x03:
//         if (tickCount >= 50) { // 500ms後
//             chipkicker.write(0);
//             kickState = IDLE;
//         }
//         break;
//     case 0x04:
//         if (tickCount >= 100) { // 1000ms後
//             dribbler.write(0.05);
//             debugled = !debugled;
//             tickCount = 0;
//             kickState = DRIBBLE_MID;
//         }
//         break;
//     case 0x05:
//         if (tickCount >= 100) { // 1000ms後
//             kickState = IDLE;
//         }
//         break;
//     default:
//         break;
//     }
// }
// }

void processData(int data) {
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
    }
    canled = !canled;
}

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//     if (can.getHcan() == hcan) {
//         can.recv(canRecvData);

//     can.send(canRecvData);
//     canled = !canled;
// }

void main_app() {
    setup();
    printf("starttt\n\r");
    charge = 0;
    readADC();
    timer.reset();
    while (1) {
        // processData(data);
        can.send(canPhotoData);
        uint8_t adcValue = readADC();
        updatePhotoDetection(adcValue);

        // debugled = !debugled;
        // HAL_Delay(300);
        printfDMA("Std:%d\n",canRecvData.stdId);
        switch (canRecvData.stdId) {
        case 0x10:
            // charge = 1;
            chargeDevice();
            // debugled = !debugled;
            break;
        case 0x11:
            straightkick();
            // debugled = !debugled;
            // straightkicker.write(0.5);
            break;
        case 0x12:
            debugled = 1;
            chipkick();
            // chipkicker.write(0.5);
            break;
        case 0x13:
            dribble();
            // dribbler.write(0.05);
            //  debugled = !debugled;
            break;

        default:
            deactivateAll();
   
            break;
        }
        // printfDMA("%d\n", canRecvData.data[0]);
        HAL_Delay(100);
    }
}

void chargeDevice() {
    if (timer.read_ms() > 5000) {
        charge = 1;
        printfDMA("charge start\n");
        debugled = 1;
        HAL_Delay(3000);
        timer.reset();
    }
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
    if (timer.read_ms() > 3000) {
        straightkicker.write(0.5);
        printfDMA("straight\n");
        HAL_Delay(200);
        timer.reset();
    }
}

void chipkick() {
    if (timer.read_ms() > 3000) {
        chipkicker.write(0.5);
        printfDMA("chipkick\n");
        HAL_Delay(200);
        timer.reset();
    }
    // chipkicker.write(0);
}

void dribble() {
    if (timer.read_ms() > 3000) {
        dribbler.write(0.1);
        printfDMA("Dribble\n");
        // debugled = !debugled;
        HAL_Delay(100);
        // dribbler.write(0.05);
        // debugled = !debugled;
        // HAL_Delay(1000);
        timer.reset();
    }
}

void deactivateAll() {
    charge = 0;
    straightkicker.write(0);
    chipkicker.write(0);
    dribbler.write(0);
}