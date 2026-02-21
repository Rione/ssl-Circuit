#ifndef __KICKER_HPP_
#define __KICKER_HPP_

#include <stdint.h>

#include "CAN.hpp"
#include "Timer.hpp"
#include "can_id.hpp"
#include "parammeter.hpp"

typedef struct {
  union {
    struct {
      bool do_direct_straight : 1;
      bool do_direct_chip : 1;
    };
    uint8_t do_direct_status;
  };
  uint8_t cap_val_estimate;
} KickerStatus;

class Kicker {
 public:
  Kicker(CANBus* can);

  void Kick(bool is_straight, uint8_t power, bool do_direct = false);

  void Charge();
  void Discharge();

  void CancelDirect(bool is_straight);

  static constexpr bool kStraight = 1;
  static constexpr bool kChip = 0;

  void UpdateCapValEstimate(uint8_t power);

  inline __attribute__((always_inline)) uint8_t GetCapValEstimate() {
    return status_.cap_val_estimate;
  }

  inline __attribute__((always_inline)) void SetCapValEstimate(uint8_t val) {
    status_.cap_val_estimate = val;
  }

 private:
  CANBus* can_;
  KickerStatus status_;
};

#endif  // __KICKER_HPP_