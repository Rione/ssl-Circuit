#include "Button.hpp"

Button::Button(GPIO_TypeDef *port, uint16_t pin) 
    : button(port, pin), 
      pressState(false), 
      isPressed(false), 
      pressedTime(0),
      releaseEvent(false) {
}

void Button::update() {
    if (read() == pressState) {
        // button is pressed
        if (isPressed == false) {
            isPressed = true;
            pressedTimer.reset();
        }
    } else {
        // button is released
        if (isPressed == true) {
            isPressed = false;
            pressedTime = pressedTimer.read_ms();
            releaseEvent = true;
        }
    }
}

bool Button::read() {
    lastButtonState = buttonState;
    buttonState = button.read();
    return buttonState;
}

bool Button::isPress() {
    return buttonState == pressState;
}

bool Button::isRelease() {
    if (!releaseEvent) {
        return false;
    }
    releaseEvent = false;
    return true;
}

uint16_t Button::readPressedTime() {
    return pressedTime;
}