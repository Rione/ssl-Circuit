#ifndef MYPWM_HPP
#define MYPWM_HPP

#include "tim.h"
#include "MyMath.hpp"

#ifdef __cplusplus
extern "C" {

class PwmOut {
  public:
    PwmOut(TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2, uint32_t channel3) : _htim(htim), channel{channel1, channel2, channel3}, maxValue(0) {
    }

    void init() {
        HAL_TIM_PWM_Start(_htim, channel[0]);
        HAL_TIM_PWM_Start(_htim, channel[1]);
        HAL_TIM_PWM_Start(_htim, channel[2]);
        HAL_TIMEx_PWMN_Start(_htim, channel[0]);
        HAL_TIMEx_PWMN_Start(_htim, channel[1]);
        HAL_TIMEx_PWMN_Start(_htim, channel[2]);
        maxValue = _htim->Init.Period;
    }

    void setMaxValue(uint32_t value) {
        maxValue = value;
    }

    void write(float duty1, float duty2, float duty3) {
        duty1 = (int)(Constrain(duty1, 0.0, 1.0) * maxValue);
        duty2 = (int)(Constrain(duty2, 0.0, 1.0) * maxValue);
        duty3 = (int)(Constrain(duty3, 0.0, 1.0) * maxValue);

        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, duty1);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, duty2);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_3, duty3);
        // printf("duty1: %f, duty2: %f, duty3: %f\n", duty1, duty2, duty3);
    }

  private:
    TIM_HandleTypeDef *_htim;
    uint32_t channel[3];
    uint32_t maxValue;

    bool usePwmPin_t;
};
};

// void pwm_init() {
//     // Start TIM1
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
// }

// void set_pwm(float duty1, float duty2, float duty3) {
//     duty1 = (int)(Constrain(duty1, 0.0, 1.0) * htim1.Init.Period);
//     duty2 = (int)(Constrain(duty2, 0.0, 1.0) * htim1.Init.Period);
//     duty3 = (int)(Constrain(duty3, 0.0, 1.0) * htim1.Init.Period);

//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty1);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty2);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty3);
// }
#endif
#endif