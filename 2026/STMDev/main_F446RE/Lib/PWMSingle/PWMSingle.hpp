#ifndef __PWMSINGLE__
#define __PWMSINGLE__

#include "tim.h"
#include "MyMath.hpp"

#ifdef __cplusplus
extern "C" {

class PwmSingleOut {
  public:
    PwmSingleOut(TIM_HandleTypeDef *htim, uint32_t channel)
        : _htim(htim), _channel(channel), _maxValue(_htim->Init.Period) {
    }

    void init() {
        HAL_TIM_PWM_Start(_htim, _channel);
        _maxValue = _htim->Init.Period;
    }

    void setMaxValue(uint32_t value) {
        _maxValue = value;
    }

    void write(float duty) {
        duty = (int)(Constrain(duty, 0.0, 1.0) * _maxValue);
        __HAL_TIM_SET_COMPARE(_htim, _channel, duty);
    }

    void operator=(float duty) {
        write(duty);
    }

  private:
    TIM_HandleTypeDef *_htim;
    uint32_t _channel;
    uint32_t _maxValue;

    bool _usePwmPin_t;
};
};

#endif
#endif