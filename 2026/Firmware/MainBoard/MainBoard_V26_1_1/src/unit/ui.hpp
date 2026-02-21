#ifndef __UI_HPP_
#define __UI_HPP_

#include <stdint.h>

#include "BufferedSerial.hpp"

typedef struct {
  union {
    struct {
      bool is_locked : 1;
      bool chip_kick : 1;
      bool straight_kick : 1;
      bool dribble : 1;
      bool charge : 1;
      bool discharge : 1;
      bool power_off : 1;
      uint8_t reserved : 1;
    };
    uint8_t flags;
  };
  union {
    struct {
      uint8_t kicker_power : 4;
      uint8_t dribbler_power : 4;
    };
    uint8_t power_data;
  };
} UiStatus;

class Robot;

class UI {
 public:
  UI(BufferedSerial* serial);

  void Recv(UiStatus& status);
  void Send(const Robot& robot);

 private:
  BufferedSerial* serial_;
};

#endif  // __UI_HPP_