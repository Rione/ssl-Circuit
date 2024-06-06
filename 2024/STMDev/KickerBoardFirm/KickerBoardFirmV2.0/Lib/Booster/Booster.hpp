#ifndef __Booster__
#define __Booster__

#include "DigitalInOut.hpp"
#include "Timer.hpp"

#include "Kicker.hpp"

class Booster {
  public:
    Booster(GPIO_TypeDef *port, uint16_t pin);

    void setChargeInterval(uint16_t interval);
    void setChargeEnable();
    void setChargeDisable();

    void update();

    void disCharge();

  private:
    DigitalOut chargePin;

    Timer chargeIntervalTimer;
    Timer chargePinTurnOffTimer;

    bool doCharge;
    bool chargePinState;

    uint16_t chargeInterval;
    uint16_t chargePinTurnOffInterval;
};

#endif