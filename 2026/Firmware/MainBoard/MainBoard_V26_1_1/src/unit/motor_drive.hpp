#ifndef __MOTOR_DRIVE_HPP_
#define __MOTOR_DRIVE_HPP_

#include <stdint.h>

#include "BufferedSerial.hpp"
#include "MyMath.hpp"
#include "parammeter.hpp"

class MotorDrive {
     public:
      MotorDrive(BufferedSerial* serial2, BufferedSerial* serial3,
                 BufferedSerial* serial5, BufferedSerial* serial6);
      void SetVel(int16_t vel_x, int16_t vel_y, int16_t vel_angle);
      void SetMotors(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
      void SendEmg();
      void GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle);

     private:
      BufferedSerial* serial2;
      BufferedSerial* serial3;
      BufferedSerial* serial5;
      BufferedSerial* serial6;
};

#endif  // __MOTOR_DRIVE_HPP_
