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

  void Kick(bool is_straight, uint8_t power, bool do_direct = false);

  void Charge();
  void Discharge();

  void CancelDirect(bool is_straight);

  static constexpr bool kStraight = 1;
  static constexpr bool kChip = 0;

  inline __attribute__((always_inline)) uint8_t GetCapValEstimate() {
    return cap_val_estimate_;
  }

  inline __attribute__((always_inline)) void SetCapValEstimate(uint8_t val) {
    cap_val_estimate_ = val;
  }

 private:
  CANBus* can_;

  uint8_t cap_val_estimate_;  // 充電の推定値
};

#endif  // __KICKER_HPP_