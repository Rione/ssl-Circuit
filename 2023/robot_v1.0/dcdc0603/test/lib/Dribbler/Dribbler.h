#ifndef DRIBBLER_H
#define DRIBBLER_H
#include "mbed.h"
#include "Servo.h"
class Dribbler {
  public:
    Dribbler(PinName dribblerPin);
    void setPower(float p);

    void dribble();
    void turnOff();
    float getPower();

  private:
    float targetPower;
    float targetPowerPrev;
    float power;
    uint8_t turnMaxInterval;
    Timer timer;
    Servo motor;
    Timer offTimer;
};

#endif
