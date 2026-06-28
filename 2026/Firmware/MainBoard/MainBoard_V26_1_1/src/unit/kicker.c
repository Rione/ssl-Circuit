#include "kicker.h"

#include <stdio.h>

void Kicker_Init(Kicker* self, CanBus* can) {
  self->can = can;
  self->kick_timer = (Timer){0};
}

void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power) {
  if (Timer_ReadMs(&self->kick_timer) < ROBOT_KICK_INTERVAL_MS) return;

  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {power, 0, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

  Timer_Reset(&self->kick_timer);
}

void Kicker_Charge(Kicker* self) {
  CanData can_data = {
      .stdId = CAN_ID_TX_CHARGE,
      .data = {0},
  };
  Can_Send(self->can, &can_data);
}

void Kicker_Discharge(Kicker* self) {
  CanData can_data = {
      .stdId = CAN_ID_TX_DISCHARGE,
      .data = {0},
  };
  Can_Send(self->can, &can_data);
}
