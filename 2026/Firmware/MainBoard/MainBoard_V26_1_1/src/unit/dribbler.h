#ifndef __DRIBBLER_H_
#define __DRIBBLER_H_

#include <stdint.h>

#include "can_bus.h"
#include "can_id.h"
#include "timer.h"

typedef union {
  struct {
    uint8_t is_detected_ball : 1;  // ボール検知（フォトセンサー）
    uint8_t is_hold_ball : 1;      // ボール保持（ドリブラー＋フォトセンサー）
    uint8_t is_new_drib : 1;       // 新機体:1
    uint8_t reserved : 5;
  };
  uint8_t data;
} DribbleStatus;

#define DRIBBLER_MAX_POWER 250        // 0以外を指定した際の送信パワー（現状は2値制御）
#define DRIBBLER_SEND_INTERVAL_MS 100  // パワー未変化時の再送間隔[ms]

typedef struct {
  CanBus *can;
  Timer send_timer;
  uint8_t power_prev;
} Dribbler;

void Dribbler_Init(Dribbler *self, CanBus *can);
void Dribbler_Send(Dribbler *self, uint8_t power, uint8_t force_send);

#endif  // __DRIBBLER_H_
