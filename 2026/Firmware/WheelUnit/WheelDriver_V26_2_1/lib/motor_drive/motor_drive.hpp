#ifndef __MOTOR_DRIVE_HPP_
#define __MOTOR_DRIVE_HPP_

#include <stdint.h>

#include "BufferedSerial.hpp"
#include "MyMath.hpp"
#include "parammeter.hpp"

class MotorDrive {
     public:
      MotorDrive(BufferedSerial* serials);
      void SetVel(int16_t vel_x, int16_t vel_y, int16_t vel_angle);
      void Send(int16_t* m);
      void Recv();
      void GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle);

      bool _emg;
      bool _ready;

     private:
      BufferedSerial* _serials[4];

      int16_t _vel_wheel_angular[4];
};

#endif  // __MOTOR_DRIVE_HPP_
