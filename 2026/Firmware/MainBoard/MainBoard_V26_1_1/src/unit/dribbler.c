#include "dribbler.h"

#include <stdio.h>

void Dribbler_Init(Dribbler* self, CanBus* can) {
  self->can = can;
  self->send_timer = (Timer){0};
  self->power_prev = 0;
}

void Dribbler_Send(Dribbler* self, uint8_t power, uint8_t force_send) {
  // 前回と同じパワーかつ強制送信でない場合、一定時間経過するまで送信しない
  if (power == self->power_prev && !force_send) {
    if (Timer_ReadMs(&self->send_timer) < DRIBBLER_SEND_INTERVAL_MS) return;
  }

  // 現状の仕様では0以外は最大出力にする
  uint8_t send_power = (power != 0) ? DRIBBLER_MAX_POWER : 0;

  CanData can_data = {
      .stdId = CAN_ID_TX_DRIBBLER,
      .data = {send_power, 0, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&self->send_timer);
  self->power_prev = power;
}
