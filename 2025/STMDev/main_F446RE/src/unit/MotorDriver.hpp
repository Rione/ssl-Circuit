#ifndef __MotorDriver__
#define __MotorDriver__

#include "CAN.hpp"
#include "MyMath.hpp"
#include "config.h"

// 単位について
// vel [m/s]
// angler [rad/s]

class MotorDriver {
  public:
    MotorDriver(CANBus *canBus);

    void init();

    void setVelocityFF(int16_t velX, int16_t velY, int16_t velAng);

    void setMotors(int16_t M0, int16_t M1, int16_t M2, int16_t M3);

    void sendEmg();
    // void setVelocity(int16_t velX, int16_t velY, float velAng);

    // void setPositionOfMotor(float pos1, float pos2, float pos3, float pos4);

    // void setPIDGain(float kp, float ki, float kd);

  private:
    CANBus *_canBus;
    
};

#endif