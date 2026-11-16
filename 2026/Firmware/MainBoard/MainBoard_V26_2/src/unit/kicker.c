#include "kicker.h"

#include <stdio.h>

void Kicker_Init(Kicker* self, CanBus* can) {
  self->can = can;
  self->kick_timer = (Timer){0};
  self->charge_timer = (Timer){0};
  self->discharge_timer = (Timer){0};
  self->direct_kick_timer = (Timer){0};
  self->direct_kick_type_prev = KICKER_DIRECT_NONE;
  self->direct_kick_power_prev = 0;
}

// 送信メールボックスに空きがない場合は送信できていないため、タイマーを進めず
// 次ループで再送を試みる(送信成否を確認できないCan_Sendの戻り値を補う)
static uint8_t Can_HasFreeMailbox(CanBus* can) {
  return HAL_CAN_GetTxMailboxesFreeLevel(Can_GetHandle(can)) > 0;
}

// 指示値(0-255)をPowerBoardへ送るPWM強さ(60-255)へ換算する
static uint8_t Kicker_ToPowerBoardPower(uint8_t power) {
  return 60 + power * (float)(255 - 60) / 255.0f;
}

void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power) {
  if (Timer_ReadMs(&self->kick_timer) < ROBOT_KICK_INTERVAL_MS) return;
  if (!Can_HasFreeMailbox(self->can)) return;

  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {Kicker_ToPowerBoardPower(power), 0, 0, 0, 0, 0, 0, 0},
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

void Kicker_SendDirectKick(Kicker* self, uint8_t type, uint8_t power) {
  if (power == 0) type = KICKER_DIRECT_NONE;
  uint8_t pb_power = (type == KICKER_DIRECT_NONE) ? 0 : Kicker_ToPowerBoardPower(power);

  uint8_t changed = type != self->direct_kick_type_prev ||
                    pb_power != self->direct_kick_power_prev;
  if (!changed &&
      Timer_ReadMs(&self->direct_kick_timer) < ROBOT_DIRECT_KICK_SIGNAL_INTERVAL_MS)
    return;
  if (!Can_HasFreeMailbox(self->can)) return;

  CanData can_data = {
      .stdId = CAN_ID_TX_DIRECT_KICK,
      .data = {type, pb_power, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

  self->direct_kick_type_prev = type;
  self->direct_kick_power_prev = pb_power;
  Timer_Reset(&self->direct_kick_timer);
}
