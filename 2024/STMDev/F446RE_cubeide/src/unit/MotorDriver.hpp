#ifndef __MotorDriver__
#define __MotorDriver__

#include "CAN.hpp"
#include "BNO055.hpp"

// 単位について
// vel [m/s]
// angler [rad/s]

class MotorDriver {
  public:
    MotorDriver(CANBus *canBus, BNO055 *bno);

    void init();

    void setVelocityFF(int16_t velX, int16_t velY, float velAng);

    // void setVelocity(int16_t velX, int16_t velY, float velAng);

    // void setPositionOfMotor(float pos1, float pos2, float pos3, float pos4);

    // void setPIDGain(float kp, float ki, float kd);

  private:
    CANBus *_canBus;
    BNO055 *_bno;
};

#endif