#ifndef BLDC_H
#define BLDC_H

#include "AS5048A.hpp"
#include "LPF.hpp"
#include "MyMath.hpp"
#include "PID.h"
#include "PWM.hpp"
#include "Timer.hpp"
#include "stdio.h"

#ifdef __cplusplus

extern "C" {

#define NOTSET -9999
#define ENCODER_DIR -1

// #define USE_TRAPEZOID_120
class BLDCMotor {
     private:
      PwmOut *pwm;
      AS5048A *encoder;
      uint8_t polePairQty;
      PID velocityPID;
      LPF velocityLPF;
      Timer timer;

      float supplyVoltage;
      float limitVoltage;  // min = -limitVoltage, max = limitVoltage

      float elAngle;                            // electrical angle
      float shAngle, shAnglePrev, shAngleZero;  // shaft angle　

      float targetVelocity;
      float limitVelocity;
      float velocity;

      double angularVelocity;

      int16_t available;
      bool debug;

     public:
      BLDCMotor(PwmOut *_pwm, AS5048A *_encoder, uint8_t _polerPairQty, float _dt);
      void init(bool setZeroPosInInit = true);

      void setAbsoluteZero(float _shAngleZero = NOTSET);
      void Diagnose();

      void setSupplyVoltage(float _supplyVoltage, float _limitVoltage);
      void setPIDGain(float _p, float _i, float _d);

      void setVelocity(float _velocity);
      void setVelocityLimit(float _limit);
      float getTargetVelocity();

      void writePwm(float _pwmA, float _pwmB, float _pwmC);

      void updateAngularVelocity();

      float updateEncoder();
      float getShaftAngle();
      float getElectricAngle();
      float getAngularVelocity();
      float getZeroPos();

      inline void setPhaseVoltage(float _Uq, float Ud, float _elAngle);
      void openLoopControl(float _A, float _elAngle);
      void drive();
};
}

#endif
#endif