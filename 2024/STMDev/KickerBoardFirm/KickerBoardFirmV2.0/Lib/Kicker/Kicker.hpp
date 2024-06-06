#ifndef __Kicker__
#define __Kicker__

#include "PWMSingle.hpp"
#include "Timer.hpp"
#include "MyMath.hpp"

class Kicker {
  public:
    Kicker(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t riseTime, uint16_t kickInterval);
    void init();
    void disable();
    void enable();

    void addRelatedKicker(Kicker *kicker);

    void kick(float power);
    void update();

    void disCharge();
    PwmSingleOut kicker;

  private:
    uint8_t _riseTime;
    uint32_t _kickInterval;

    Timer _riseTimer;
    Timer _intervalTimer;

    bool available;
    bool isDischarging;

    Kicker *relatedKicker;
};

#endif