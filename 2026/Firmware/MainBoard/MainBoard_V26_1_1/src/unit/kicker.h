#ifndef __KICKER_H_
#define __KICKER_H_

#include <stdint.h>

#include "can_bus.h"
#include "can_id.h"
#include "parammeter.h"
#include "timer.h"

typedef struct {
  union {
    struct {
      uint8_t do_direct_straight : 1;
      uint8_t do_direct_chip : 1;
    };
    uint8_t do_direct_status;
  };
  uint16_t cap_val;
  uint8_t done_charge;
} KickerStatus;

#define KICKER_STRAIGHT 1
#define KICKER_CHIP 0

typedef struct {
  CanBus* can;
  KickerStatus status;
} Kicker;

void Kicker_Init(Kicker* self, CanBus* can);
void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power);
void Kicker_Charge(Kicker* self);
void Kicker_Discharge(Kicker* self);
void Kicker_CancelDirect(Kicker* self);
uint16_t Kicker_GetCapVal(Kicker* self);

#endif  // __KICKER_H_
