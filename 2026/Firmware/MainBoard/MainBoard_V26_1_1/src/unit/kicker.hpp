#ifndef __KICKER_HPP_
#define __KICKER_HPP_

#include <stdint.h>

#include "CAN.hpp"
#include "Timer.hpp"
#include "can_id.hpp"
#include "parammeter.hpp"

class Kicker {
 public:
  Kicker(CANBus* can);

  void Kick(bool type, uint8_t power, bool do_direct = false);
  void ChargeControl(bool type);
  void StopDirect(bool type);

#define STRAIGHT 1
#define CHIP 0

 private:
  CANBus* can_;

  Timer kick_interval_timer_;
};

#endif  // __KICKER_HPP_