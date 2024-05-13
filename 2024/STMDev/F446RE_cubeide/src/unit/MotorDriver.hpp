#ifndef __MotorDriver__
#define __MotorDriver__

#include "CAN.hpp"
#include "BNO055.hpp"
class MotorDriver {
  public:
    MotorDriver(CANBus *_canBus, BNO055 *bno);

    void init();

    void setVelocityFF(float velX, float velY, float velZ);

    void setVelocity(float velX, float velY, float velZ);

    void setPositionOfMotor(float pos1, float pos2, float pos3, float pos4);

    void setPIDGain(float kp, float ki, float kd);

    // canBus,imu

    // - VelX,VelY,VelZ
    //
  private:
    CANBus *canBus;
    BNO055 *bno;
};

#endif