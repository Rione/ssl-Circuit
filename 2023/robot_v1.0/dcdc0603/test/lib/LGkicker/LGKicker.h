#ifndef LGKIKCER_H
#define LGKIKCER_H
#include "mbed.h"
class LGKicker {
  public:
    LGKicker(PinName KickerPin);
    bool Kick(void);
    void setPower(float p);
    void discharge(void);
  private:
    PwmOut Kicker; // DigitalOut Kicker;
    Timer timer;
    bool enabled;
    float power;
    Timeout Kicker_timeout;
    Timeout KickerIsRedey;
    void flip(void);
    void flipOn(void);
};

#endif
