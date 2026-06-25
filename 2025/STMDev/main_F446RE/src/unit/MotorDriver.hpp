#ifndef __MotorDriver__
#define __MotorDriver__

#include "CAN.hpp"
#include "MyMath.hpp"
#include "config.h"

// 単位について
// vel [m/s]
// angler [rad/s]
#define GEAR_RATIO (56.0 / 15.0)                                                                // gear ratio 56：15
#define INV_GEAR_RATIO (15.0 / 56.0)                                                            // motor → wheel
#define WHEEL_OMEGA_SCALE_MIN (-32100)
#define WHEEL_OMEGA_SCALE_MAX (32000)
#define WHEEL_DIAMETER 54                                                                       // wheel diameter 54mm
#define WHEEL_BASE_DIAMETER 170                                                                 // robot wheel base diameter
#define ROBOT_SPIN_TO_MOTOR_ROTATE_RATIO ((WHEEL_BASE_DIAMETER / WHEEL_DIAMETER) * GEAR_RATIO)  // robot spins → wheelRotate → motorRotate
#define VELOCITY_XY_TO_MOTOR_ROTATE_RATIO (GEAR_RATIO / (WHEEL_DIAMETER / 2))                   // robot moves XY → wheelRotate → motoRotate

class MotorDriver {
     public:
      MotorDriver(CANBus *canBus);

      void init();

      void setVelocityFF(int16_t velX, int16_t velY, int16_t velAng);

      void setMotors(int16_t M0, int16_t M1, int16_t M2, int16_t M3);

      void getVelocity(int16_t *velX, int16_t *velY, int16_t *motorAngulerVelocity);

      static int16_t motorToWheelScaled(int16_t motor_omega);

      void sendEmg();
      // void setVelocity(int16_t velX, int16_t velY, float velAng);

      // void setPositionOfMotor(float pos1, float pos2, float pos3, float pos4);

      // void setPIDGain(float kp, float ki, float kd);

     private:
      CANBus *_canBus;
};

#endif