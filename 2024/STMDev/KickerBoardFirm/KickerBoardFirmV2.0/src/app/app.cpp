#include "app.h"
#include "DigitalInOut.hpp"

#include "PWMSingle.hpp"
#include "PWMSingleN.hpp"
#include "CAN.hpp"
#include "DMAStream.h"
#include "Timer.hpp"
#include "adc.h"

#include "Button.hpp"
#include "Kicker.hpp"
#include "Booster.hpp"

CANBus can(&hcan, 0x100);
CANBus::CANData canRecvData;

Button sw1(SW1_GPIO_Port, SW1_Pin);
Button sw2(SW2_GPIO_Port, SW2_Pin);

Kicker straightKicker(&htim15, TIM_CHANNEL_2, 50, 1000);
Kicker chipKicker(&htim3, TIM_CHANNEL_2, 50, 1000);

Booster booster(CHARGE_GPIO_Port, CHARGE_Pin);

Timer canTransmitIntervalTimer;

void TimInterrupt500hz() {
    sw1.update();
    sw2.update();

    straightKicker.update();
    chipKicker.update();

    booster.update();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
        switch (canRecvData.stdId) {
        case 0x10: // charge
            booster.setChargeEnable();
            break;
        case 0x11: // kick
            straightKicker.kick((float)(canRecvData.data[0]) / 100);
            break;
        case 0x12: // chip kick
            chipKicker.kick((float)(canRecvData.data[0]) / 100);
            break;
        case 0x13:
            // dribbleFlag = true;
            break;
        case 0x14:
            // dribbleStopFlag = true;
            break;
        default:
            break;
        }
    }
}

uint16_t readPhotoADC() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return HAL_ADC_GetValue(&hadc1);
}

void setup() {
    can.init();
    // kickers
    straightKicker.init();
    chipKicker.init();

    // add related kicker for mutual exclusion
    straightKicker.addRelatedKicker(&chipKicker);
    chipKicker.addRelatedKicker(&straightKicker);
    // booster
    booster.setChargeInterval(1000);
    booster.setChargeEnable();
    printf("Hello World\n");
}

void main_app() {
    setup();
    while (1) {
        uint16_t photoValue = readPhotoADC();
        printf("sw1:%4dms sw2:%4dms Photo:%4d\n", sw1.readPressedTime(), sw2.readPressedTime(), photoValue);

        if (sw1.isRelease()) {
            if (sw1.readPressedTime() > 500) {
                printf("DISCHARGE\n");
                booster.setChargeDisable();
                HAL_Delay(500);
                straightKicker.disCharge();
            } else {
                chipKicker.kick(0.5);
                printf("CHIP\n");
            }
        }

        if (sw2.isRelease()) {
            if (sw2.readPressedTime() > 500) {
                printf("CHARGE\n");
                HAL_Delay(500);
                booster.setChargeEnable();
            } else {
                straightKicker.kick(0.5);
                printf("STRAIGHT\n");
            }
        }

        if (canTransmitIntervalTimer.read_ms() > 10) {
            CANBus::CANData canPhotoData = {
                .stdId = 0x123,
                .data = {(uint8_t)(photoValue & 0xFF),
                         (uint8_t)((photoValue >> 8) & 0xFF)}};
            can.send(canPhotoData);
            canTransmitIntervalTimer.reset();
        }
    }
}
