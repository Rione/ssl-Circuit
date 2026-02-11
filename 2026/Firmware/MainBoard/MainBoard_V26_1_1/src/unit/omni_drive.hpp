#ifndef __OMNI_DRIVE_HPP_
#define __OMNI_DRIVE_HPP_

#include <stdint.h>

#include "BufferedSerial.hpp"
#include "MyMath.hpp"
#include "parammeter.hpp"

class OmniDrive {
 public:
  OmniDrive(BufferedSerial* serials);
  void SetVel(int16_t vel_x, int16_t vel_y, int16_t vel_angle);
  void Send(int16_t* m);
  void Recv();
  void GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle);

  bool emg_;
  bool ready_;

 private:
  BufferedSerial* serials_[4];

  int16_t vel_wheel_angular_[4];
};

#endif  // __OMNI_DRIVE_HPP_
