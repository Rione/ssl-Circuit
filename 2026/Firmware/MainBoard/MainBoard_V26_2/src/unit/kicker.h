#ifndef __KICKER_H_
#define __KICKER_H_

#include <stdint.h>

#include "can_bus.h"
#include "can_id.h"
#include "parammeter.h"
#include "timer.h"

typedef struct {
  uint16_t cap_val;
  uint8_t done_charge;
} KickerStatus;

#define KICKER_STRAIGHT 1
#define KICKER_CHIP 0

// ダイレクトキック待機指示の種別(CAN 0x15 data[0])
#define KICKER_DIRECT_NONE 0
#define KICKER_DIRECT_STRAIGHT 1
#define KICKER_DIRECT_CHIP 2

typedef struct {
  CanBus* can;
  Timer kick_timer;
  Timer charge_timer;
  Timer discharge_timer;
  Timer direct_kick_timer;
  uint8_t direct_kick_type_prev;
  uint8_t direct_kick_power_prev;
} Kicker;

void Kicker_Init(Kicker* self, CanBus* can);
void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power);
void Kicker_Charge(Kicker* self);
void Kicker_Discharge(Kicker* self);
// PowerBoardへダイレクトキックの待機を指示する(値の変化時と周期で送信)。
// フォトセンサ反応の検出とキックはPowerBoardがDribblerのCANフレームを直接見て行う。
void Kicker_SendDirectKick(Kicker* self, uint8_t type, uint8_t power);

#endif  // __KICKER_H_
