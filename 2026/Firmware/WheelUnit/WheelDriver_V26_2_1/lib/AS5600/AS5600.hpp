#ifndef __AS5600__
#define __AS5600__

#ifdef __cplusplus

#include "main.h"

#define PI 3.1415926535897932384626433832795

class AS5600 {
  public:
    AS5600(ADC_HandleTypeDef *hadc, uint32_t channel);
    void init();
    uint16_t getRawValue();
    float getAngleRad();
    float getAngleDeg();

  private:
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
};

#endif
#endif
