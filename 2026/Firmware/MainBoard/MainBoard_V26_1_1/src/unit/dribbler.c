#include "dribbler.h"

#include <stdio.h>

void Dribbler_Init(Dribbler* self, CanBus* can) {
  self->can = can;
}

void Dribbler_Send(Dribbler* self, uint8_t power, uint8_t force_send) {
  static Timer timer = {0};
  static uint8_t dribble_power_prev = 0;

  // 前回と同じパワーかつ強制送信でない場合、100ms経過するまで送信しない
  if (power == dribble_power_prev && !force_send) {
    if (Timer_ReadMs(&timer) < 100) return;
  }

  // 現状の仕様では0以外は最大出力(100)にする
  uint8_t send_power = (power != 0) ? 250 : 0;

  CanData can_data = {
      .stdId = CAN_ID_TX_DRIBBLER,
      .data = {send_power, 0, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&timer);
  dribble_power_prev = power;
}
