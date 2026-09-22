#ifndef __ANALOGIN_HPP__
#define __ANALOGIN_HPP__

#include "stm32g4xx_hal.h"

class AnalogIn {
  public:
    AnalogIn(ADC_HandleTypeDef *hadc, uint32_t channel)
        : hadc_(hadc) {
        sConfig_ = {0}; // 全メンバをゼロ初期化
        sConfig_.Channel = channel;
        sConfig_.Rank = ADC_REGULAR_RANK_1;
        sConfig_.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;
        sConfig_.SingleDiff = ADC_SINGLE_ENDED;
        sConfig_.OffsetNumber = ADC_OFFSET_NONE;
        sConfig_.Offset = 0;
    }

    ~AnalogIn() {}

    inline uint16_t read(uint32_t timeout = 100) {
        HAL_ADC_ConfigChannel(hadc_, &sConfig_);
        HAL_ADC_Start(hadc_);
        HAL_ADC_PollForConversion(hadc_, timeout); // ADC変換終了を待機
        HAL_ADC_Stop(hadc_);
        return HAL_ADC_GetValue(hadc_);
    }

  private:
    ADC_HandleTypeDef *hadc_;
    ADC_ChannelConfTypeDef sConfig_;
};

#endif