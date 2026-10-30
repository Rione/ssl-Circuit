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
    // 前回の呼び出し以降に離されたか(呼ぶと消える)。updateは1kHz割り込みで回っているので、
    // メインループの周期とずれても離した瞬間を取りこぼさないよう、離した事実を割り込み側で保持する
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

    volatile uint16_t pressedTime;
    volatile bool releaseEvent;
    Timer pressedTimer;
};

#endif