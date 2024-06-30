#ifndef __Booster__
#define __Booster__

#include "DigitalInOut.hpp"
#include "Timer.hpp"

#include "Kicker.hpp"

class Booster {
  public:
    Booster(GPIO_TypeDef *chargePort, uint16_t chargepin, GPIO_TypeDef *donePort, uint16_t donePin);

    void setChargeInterval(uint16_t interval);
    void setChargeEnable();
    void setChargeDisable();

    void update();
    bool getDone();

  private:
    DigitalOut chargePin;
    DigitalIn donePin;

    Timer chargeIntervalTimer;
    Timer watchTurnOffIntervalTimer;

    bool doCharge;
    bool chargePinState;

    uint8_t chargeDoneCount;

    uint16_t chargeInterval;
    uint16_t chargePinTurnOffInterval;
};

#endif