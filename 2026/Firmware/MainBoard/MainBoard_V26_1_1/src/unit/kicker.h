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
  uint8_t cap_val_estimate;
} KickerStatus;

#define KICKER_STRAIGHT 1
#define KICKER_CHIP     0

typedef struct {
  CanBus     *can;
  KickerStatus status;
} Kicker;

void    Kicker_Init(Kicker *self, CanBus *can);
void    Kicker_Kick(Kicker *self, uint8_t is_straight, uint8_t power, uint8_t do_direct);
void    Kicker_Charge(Kicker *self);
void    Kicker_Discharge(Kicker *self);
void    Kicker_CancelDirect(Kicker *self, uint8_t is_straight);
void    Kicker_UpdateCapValEstimate(Kicker *self, uint8_t power);
uint8_t Kicker_GetCapValEstimate(Kicker *self);
void    Kicker_SetCapValEstimate(Kicker *self, uint8_t val);

#endif  // __KICKER_H_
