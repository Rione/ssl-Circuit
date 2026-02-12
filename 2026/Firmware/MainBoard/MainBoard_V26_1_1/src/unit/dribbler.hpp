#ifndef __DRIBBLER_HPP_
#define __DRIBBLER_HPP_

#include <stdint.h>

#include "CAN.hpp"
#include "Timer.hpp"
#include "can_id.hpp"

class Dribbler {
 public:
  Dribbler(CANBus* can);

  void Send(uint8_t power, bool force_send = false);

 private:
  CANBus* can_;
};

#endif  // __DRIBBLER_HPP_
