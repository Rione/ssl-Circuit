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

Booster booster(CHARGE_GPIO_Port, CHARGE_Pin, &straightKicker);

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
            straightKick();
            break;
        case 0x12: // chip kick
            chipKick();
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

void chipKick() {
    straightKicker.disable();
    chipKicker.kick(1.0);
    straightKicker.enable();
}

void straightKick() {
    chipKicker.disable();
    straightKicker.kick(1.0);
    chipKicker.enable();
}

void setup() {
    can.init();
    // kickers
    straightKicker.init();
    chipKicker.init();
    // booster
    booster.setChargeInterval(1000);
    booster.setChargeEnable();
    printf("Hello World\n");
}

void main_app() {
    setup();
    while (1) {
        printf("sw1:%d sw2:%d\n", sw1.readPressedTime(), sw2.readPressedTime());

        if (sw1.isRelease()) {
            if (sw1.readPressedTime() > 500) {
                printf("DISCHARGE\n");
                HAL_Delay(500);
                booster.setChargeDisable();
                straightKicker.disCharge();
            } else {
                chipKick();
            }
        }

        if (sw2.isRelease()) {
            if (sw2.readPressedTime() > 500) {
                printf("CHARGE\n");
                HAL_Delay(500);
                booster.setChargeEnable();
            } else {
                straightKick();
            }
        }
    }
}
