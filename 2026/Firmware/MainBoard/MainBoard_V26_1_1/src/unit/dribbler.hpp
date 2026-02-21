#ifndef __DRIBBLER_HPP_
#define __DRIBBLER_HPP_

#include <stdint.h>

#include "CAN.hpp"
#include "Timer.hpp"
#include "can_id.hpp"

typedef union {
  struct {
    bool is_detected_ball : 1;  // ボール検知（フォトセンサーの検知）2024版のis_hold_ball
    bool is_hold_ball : 1;      // ボール保持（ドリブラーの検知とフォトセンサーの検知の論理積）
    bool is_new_drib : 1;       // 新機体:1
    uint8_t reserved : 5;
  };
  uint8_t data;
} DribbleStatus;

class Dribbler {
 public:
  Dribbler(CANBus* can);

  void Send(uint8_t power, bool force_send = false);

 private:
  CANBus* can_;
};

#endif  // __DRIBBLER_HPP_
