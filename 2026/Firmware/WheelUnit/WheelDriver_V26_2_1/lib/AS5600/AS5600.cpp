#include "AS5600.hpp"

AS5600::AS5600(ADC_HandleTypeDef *hadc, uint32_t channel) 
    : hadc(hadc), channel(channel) {
    init();
}

void AS5600::init() {
    // ADC チャンネルを設定
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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
    // ADC から値を読む
    if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK) {
        return HAL_ADC_GetValue(hadc);
    }
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
