

#include "Booster.hpp"

Booster::Booster(GPIO_TypeDef *port, uint16_t pin, Kicker *straightKicker)
    : chargePin(port, pin), straightKicker(straightKicker), doCharge(false), chargePinState(false), chargeInterval(5000), chargePinTurnOffInterval(2500) {
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
        // charge
        if (chargePinState == false && chargeIntervalTimer.read_ms() >= chargeInterval) {
            // turn on charge pin
            chargePin.write(true);
            chargePinState = true;

            chargePinTurnOffTimer.reset();
            printf("charge pin on\n");
        }

        if (chargePinState == true && chargePinTurnOffTimer.read_ms() >= chargePinTurnOffInterval) {
            // turn off charge pin
            chargePin.write(false);
            chargePinState = false;

            chargeIntervalTimer.reset();
            printf("charge pin off\n");
        }
    }
}
