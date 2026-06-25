#ifndef __Button__
#define __Button__

#include "DigitalInOut.hpp"
#include "Timer.hpp"

class Button {
  public:
    Button(GPIO_TypeDef *port, uint16_t pin);
    void update();
    bool read();
    bool isPress();
    bool isRelease();
    uint16_t readPressedTime();

    void setPressState(bool state) {
        pressState = state;
    }

  private:
    DigitalIn button;
    bool pressState;
    bool buttonState;
    bool lastButtonState;
    bool isPressed;

    uint16_t pressedTime;
    Timer pressedTimer;
};

#endif