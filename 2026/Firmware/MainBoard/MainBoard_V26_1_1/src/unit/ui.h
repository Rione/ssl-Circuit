#ifndef __UI_H_
#define __UI_H_

#include <stdint.h>

#include "serial.h"

typedef struct {
  union {
    struct {
      uint8_t is_locked    : 1;
      uint8_t chip_kick    : 1;
      uint8_t straight_kick: 1;
      uint8_t dribble      : 1;
      uint8_t charge       : 1;
      uint8_t discharge    : 1;
      uint8_t power_off    : 1;
      uint8_t reserved     : 1;
    };
    uint8_t flags;
  };
  union {
    struct {
      uint8_t kicker_power  : 4;
      uint8_t dribbler_power: 4;
    };
    uint8_t power_data;
  };
} UiStatus;

struct Robot;  // 前方宣言（循環インクルード回避）

typedef struct {
  Serial *serial;
} UI;

void UI_Init(UI *self, Serial *serial);
void UI_Recv(UI *self, UiStatus *status);
void UI_Send(UI *self, const struct Robot *robot);

#endif  // __UI_H_
