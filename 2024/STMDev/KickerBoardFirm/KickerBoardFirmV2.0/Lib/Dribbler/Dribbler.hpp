#ifndef __Dribbler__
#define __Dribbler__

#include "PWMSingleN.hpp"

class Dribbler {
  public:
    Dribbler(TIM_HandleTypeDef *htim, uint32_t channel);
    void init();
    void run();
    void stop();
    void write(float power);
    PwmSingleOutN pin;
};

#endif