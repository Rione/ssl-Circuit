#include "app.h"

#include "CAN.hpp"
#include "Timer.hpp"

#include "Button.hpp"
#include "Kicker.hpp"
#include "Booster.hpp"
#include "Dribbler.hpp"

#include "DMAStream.h"

CANBus can(&hcan, 0x100);
CANBus::CANData canRecvData;

DigitalOut ledCan(CAN_LED_GPIO_Port, CAN_LED_Pin);
DigitalOut ledDebug(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);

Button sw1(SW1_GPIO_Port, SW1_Pin);
Button sw2(SW2_GPIO_Port, SW2_Pin);

Kicker straightKicker(&htim15, TIM_CHANNEL_2, 50, 1000);
Kicker chipKicker(&htim3, TIM_CHANNEL_2, 50, 1000);

Booster booster(CHARGE_GPIO_Port, CHARGE_Pin, DONE_GPIO_Port, DONE_Pin);
DigitalIn donePin(DONE_GPIO_Port, DONE_Pin);

Dribbler dribbler(&htim1, TIM_CHANNEL_2);

Timer canTransmitIntervalTimer;

bool dischargeFlag = false;

volatile uint16_t photoSensorThreshold = 750;
uint16_t photoValue = 0;
bool isHoldBall = false;

union {
    struct {
        bool straight : 1;
        bool chip : 1;
    };
    uint8_t status;
} doDirectStatus;

uint16_t readPhotoADC() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return HAL_ADC_GetValue(&hadc1);
}

void TimInterrupt500hz() {
    sw1.update();
    sw2.update();

    straightKicker.update();
    chipKicker.update();

    booster.update();

    photoValue = readPhotoADC();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
        switch (canRecvData.stdId) {
        case 0x09: // 15: PhotoThreshold
            photoSensorThreshold = (uint16_t)(canRecvData.data[0] | (canRecvData.data[1] << 8));
            break;
        case 0x10: // 16: charge Enable
            booster.setChargeEnable();
            break;
        case 0x11: // 17: charge Disable
            dischargeFlag = true;
            break;
        case 0x12: // 18: kick
            // data[1] == 0x00: kick , data[1] == 0xFF: doDirectKick)
            if (canRecvData.data[1] == 0xFF) { // doDirectKick
                straightKicker.setDirectKick(true, (float)canRecvData.data[0] / 100.0);
                chipKicker.disableDirectKick();
            } else {
                straightKicker.disableDirectKick();
                straightKicker.kick((float)canRecvData.data[0] / 100.0);
            }
            break;
        case 0x13: // 19: chip kick
            // data[1] == 0x00: kick , data[1] == 0xFF: doDirectKick)
            if (canRecvData.data[1] == 0xFF) { // doDirectKick
                chipKicker.setDirectKick(true, (float)canRecvData.data[0] / 100.0);
                straightKicker.disableDirectKick();
            } else {
                chipKicker.disableDirectKick();
                chipKicker.kick((float)canRecvData.data[0] / 100.0);
            }
            break;
        case 0x14: // 20: dribbler run
            dribbler.write((float)canRecvData.data[0] / 100.0);
            break;
        case 0x15: // 21: dribbler stop
            dribbler.stop();
            break;
        case 0x16: // 22: debug led
            ledDebug = (canRecvData.data[0] != 0 ? 1 : 0);
            break;
        default:
            break;
        }
    }
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

    for (size_t i = 0; i < 6; i++) {
        ledDebug = !ledDebug;
        ledCan = !ledCan;
        HAL_Delay(50);
    }

    printf("Hello World\n");
}

uint8_t getCAN_TEC() {
    // CAN エラーステータスレジスタ(CAN_ESR)
    // 250回エラーが起きると止まる
    return ((hcan.Instance->ESR) >> 16) & 0xFF;
}

void main_app() {
    setup();
    while (1) {
        ledDebug = donePin.read();
        isHoldBall = photoValue < photoSensorThreshold;
        if (dischargeFlag) {
            printf("DISCHARGE\n");
            booster.setChargeDisable();
            HAL_Delay(500);
            straightKicker.disCharge();
            dischargeFlag = false;
        }

        if (straightKicker.directKick(isHoldBall)) {
            CANBus::CANData canData = {
                .stdId = 0x12,
                .data = {(uint8_t)(straightKicker.getDirectKickPower() * 100), 0xFF, 0, 0, 0, 0, 0, 0xFF},
            };
            can.send(canData);
        }
        if (chipKicker.directKick(isHoldBall)) {
            CANBus::CANData canData = {
                .stdId = 0x13,
                .data = {(uint8_t)(chipKicker.getDirectKickPower() * 100), 0xFF, 0, 0, 0, 0, 0, 0xFF},
            };
            can.send(canData);
        }

        // if (sw1.isRelease()) {
        //     if (sw1.readPressedTime() > 500) {
        //         printf("DISCHARGE\n");
        //         booster.setChargeDisable();
        //         HAL_Delay(500);
        //         straightKicker.disCharge();
        //     } else {
        //         chipKicker.kick(0.5);
        //         printf("CHIP\n");
        //     }
        // }

        // if (sw2.isRelease()) {
        //     if (sw2.readPressedTime() > 500) {
        //         printf("CHARGE\n");
        //         HAL_Delay(500);
        //         booster.setChargeEnable();
        //     } else {
        //         straightKicker.kick(0.5);
        //         printf("STRAIGHT\n");
        //     }
        // }

        doDirectStatus.straight = straightKicker.getDoDirecStatus();
        doDirectStatus.chip = chipKicker.getDoDirecStatus();

        if (canTransmitIntervalTimer.read_ms() > 10) {
            bool done = booster.getDone();
            printf("sw1:%4dms sw2:%4dms Photo:%4d Done:%d directSt:%d directCh:%d TEC:%d\n", sw1.readPressedTime(), sw2.readPressedTime(), photoValue, done, doDirectStatus.straight, doDirectStatus.chip, getCAN_TEC());
            CANBus::CANData canPhotoData = {
                .stdId = 0x123,
                .data = {
                    (uint8_t)(photoValue & 0xFF),
                    (uint8_t)((photoValue >> 8) & 0xFF),
                    (uint8_t)(done ? 255 : 0),
                    (uint8_t)(booster.getDoChargeState() ? 255 : 0),
                    (uint8_t)(doDirectStatus.status),
                    (uint8_t)(photoSensorThreshold & 0xFF),
                    (uint8_t)((photoSensorThreshold >> 8) & 0xFF),
                }};
            can.send(canPhotoData);
            canTransmitIntervalTimer.reset();
        }
    }
}
