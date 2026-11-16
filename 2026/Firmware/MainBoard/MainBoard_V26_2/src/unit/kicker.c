#include "kicker.h"

#include <stdio.h>

void Kicker_Init(Kicker* self, CanBus* can) {
  self->can = can;
  self->kick_timer = (Timer){0};
  self->charge_timer = (Timer){0};
  self->discharge_timer = (Timer){0};
}

// 送信メールボックスに空きがない場合は送信できていないため、タイマーを進めず
// 次ループで再送を試みる(送信成否を確認できないCan_Sendの戻り値を補う)
static uint8_t Can_HasFreeMailbox(CanBus* can) {
  return HAL_CAN_GetTxMailboxesFreeLevel(Can_GetHandle(can)) > 0;
}

void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power) {
  if (Timer_ReadMs(&self->kick_timer) < ROBOT_KICK_INTERVAL_MS) return;
  if (!Can_HasFreeMailbox(self->can)) return;

  power = 60 + power * (float)(255 - 60) / 255.0f;

  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {power, 0, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&self->kick_timer);
}

void Kicker_Charge(Kicker* self) {
  if (Timer_ReadMs(&self->charge_timer) < ROBOT_KICKER_SIGNAL_INTERVAL_MS)
    return;
  if (!Can_HasFreeMailbox(self->can)) return;

  CanData can_data = {
      .stdId = CAN_ID_TX_CHARGE,
      .data = {0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&self->charge_timer);
}

void Kicker_Discharge(Kicker* self) {
  if (Timer_ReadMs(&self->discharge_timer) < ROBOT_KICKER_SIGNAL_INTERVAL_MS)
    return;
  if (!Can_HasFreeMailbox(self->can)) return;

  CanData can_data = {
      .stdId = CAN_ID_TX_DISCHARGE,
      .data = {0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&self->discharge_timer);
}
