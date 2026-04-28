#include "AS5600.hpp"

AS5600::AS5600(ADC_HandleTypeDef *hadc, uint32_t channel) 
    : hadc(hadc), channel(channel) {}

void AS5600::init() {
    HAL_ADC_Stop(hadc);

    // ADC チャンネルを設定
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // ADC スタート
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        Error_Handler();
    }
}

uint16_t AS5600::getRawValue() {
    // 単発変換のため、読むたびに変換開始する
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK) {
        uint16_t value = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return value;
    }

    HAL_ADC_Stop(hadc);
    return 0;
}

float AS5600::getAngleRad() {
    // AS5600 は 12bit (0-4095) で 0-2π を表現
    uint16_t raw = getRawValue();
    return (float)raw / 4095.0f * 2.0f * PI;
}

float AS5600::getAngleDeg() {
    // AS5600 は 12bit (0-4095) で 0-360° を表現
    uint16_t raw = getRawValue();
    return (float)raw / 4095.0f * 360.0f;
}
