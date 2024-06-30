

#include "Booster.hpp"

Booster::Booster(GPIO_TypeDef *chargePort, uint16_t chargepin, GPIO_TypeDef *donePort, uint16_t donePin)
    : chargePin(chargePort, chargepin), donePin(donePort, donePin), doCharge(false), chargePinState(false), chargeDoneCount(false), chargeInterval(500), chargePinTurnOffInterval(2500) {
}

void Booster::setChargeInterval(uint16_t interval) {
    chargeInterval = interval;
}

void Booster::setChargeEnable() {
    doCharge = true;
}

void Booster::setChargeDisable() {
    doCharge = false;
    chargePin.write(false);
    chargePinState = false;
}

void Booster::update() {
    if (doCharge) {
        if (chargePin == false && chargeIntervalTimer.read_ms() >= chargeInterval) {
            chargePin.write(true); // 充電スタート
            printf("try charge...\n");
            watchTurnOffIntervalTimer.reset();
        } else {
            if (!donePin == true) { // doneの時
                chargePin.write(false);
                chargeIntervalTimer.reset();
                uint16_t time = watchTurnOffIntervalTimer.read_ms();
                if (time < 100) {
                    printf("already Full!!\n");
                } else {
                    printf("charge Done!!! %dms\n", time);
                    chargeDoneCount++;
                }
            }
        }
    }
}

bool Booster::getDone() {
    static uint8_t lastState = 0;
    static bool isChargeDone = false;
    if (chargeDoneCount != lastState) {
        isChargeDone = true;
    } else {
        isChargeDone = false;
    }
    lastState = chargeDoneCount;
    return isChargeDone;
}