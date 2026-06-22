#ifndef __AS5048A__
#define __AS5048A__

#ifdef __cplusplus

extern "C" {

#include "main.h"
#include <stdio.h>
#include "spi.h"
#include "MyMath.hpp"

class AS5048A {
  public:
    AS5048A(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
    uint16_t get(uint16_t addr);
    float getAngleRad();
    float getAngleDeg();

  private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    uint8_t spiCalcEvenParity(uint16_t value);
    void wait_ns(uint32_t mircos);
};
}

#endif
#endif