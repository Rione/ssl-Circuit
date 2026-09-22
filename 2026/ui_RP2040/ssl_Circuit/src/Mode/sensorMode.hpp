#ifndef _SENSOR_MODE_
#define _SENSOR_MODE_

#include "Mode.hpp"

class SensorMode : public Mode {
  public:
    using Mode::Mode;

    void displaySet() override;
    void determine() override;

  private:
    void drawUI();
    void updateBallSensor(bool force = false);
    void updateMotors(bool force = false);

    bool prevBallSensor = false;
    uint8_t prevMotorStatus = 0;
};

#endif
