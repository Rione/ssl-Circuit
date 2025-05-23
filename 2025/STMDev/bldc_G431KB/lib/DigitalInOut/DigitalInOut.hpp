#ifndef __DigitalInOut__
#define __DigitalInOut__

#include "main.h"

#ifdef __cplusplus

extern "C" {

class DigitalOut {
  public:
    DigitalOut(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
    void write(bool value) {
        HAL_GPIO_WritePin(port, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void operator=(bool value) {
        HAL_GPIO_WritePin(port, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    operator bool() {
        return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
    }

  private:
    GPIO_TypeDef *port;
    uint16_t pin;
};

class DigitalIn {
  public:
    DigitalIn(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) {
    }
    bool read() {
        return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
    }

    operator bool() {
        return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
    }

  private:
    GPIO_TypeDef *port;
    uint16_t pin;
};
}

#endif
#endif