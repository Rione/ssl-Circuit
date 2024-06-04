#include "app.h"

#include "CAN.hpp"
#include "Timer.hpp"

#include "Button.hpp"
#include "Kicker.hpp"
#include "Booster.hpp"
#include "Dribbler.hpp"

CANBus can(&hcan, 0x100);
CANBus::CANData canRecvData;

Button sw1(SW1_GPIO_Port, SW1_Pin);
Button sw2(SW2_GPIO_Port, SW2_Pin);

Kicker straightKicker(&htim15, TIM_CHANNEL_2, 50, 1000);
Kicker chipKicker(&htim3, TIM_CHANNEL_2, 50, 1000);

Booster booster(CHARGE_GPIO_Port, CHARGE_Pin);

Dribbler dribbler(&htim1, TIM_CHANNEL_2);

Timer canTransmitIntervalTimer;

bool dischargeFlag = false;

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
        case 0x10: // 16: charge Enable
            booster.setChargeEnable();
            break;
        case 0x11: // 17: charge Disable
            dischargeFlag = true;
            break;
        case 0x12: // 18: kick
            straightKicker.kick((float)canRecvData.data[0] / 100.0);
            break;
        case 0x13: // 19: chip kick
            chipKicker.kick((float)canRecvData.data[0] / 100.0);
            break;
        case 0x14: // 20: dribbler run
            dribbler.write((float)canRecvData.data[0] / 100.0);
            break;
        case 0x15: // 21: dribbler stop
            dribbler.stop();
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
    // booster.setChargeEnable(); // charge start

    // dribbler
    dribbler.init();
    printf("Hello World\n");
}

void main_app() {
    setup();
    while (1) {
        uint16_t photoValue = readPhotoADC();
        printf("sw1:%4dms sw2:%4dms Photo:%4d\n", sw1.readPressedTime(), sw2.readPressedTime(), photoValue);
        if (dischargeFlag) {
            printf("DISCHARGE\n");
            booster.setChargeDisable();
            HAL_Delay(500);
            straightKicker.disCharge();
            dischargeFlag = false;
        }

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
