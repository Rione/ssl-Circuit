#include "kicker.h"

#include <stdio.h>

void Kicker_Init(Kicker* self, CanBus* can) {
  self->can = can;
  self->status.do_direct_status = 0;
  self->status.cap_val = 0;
}

void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power) {
  static Timer timer = {0};
  if (Timer_ReadMs(&timer) < ROBOT_KICK_INTERVAL_MS) return;

  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {power, 0, 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);
  printf("Kicker_Kick: is_straight=%d, power=%d\n", is_straight, power);

  Timer_Reset(&timer);
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

void Kicker_CancelDirect(Kicker* self) {
  self->status.do_direct_status = 0;
}

uint16_t Kicker_GetCapVal(Kicker* self) {
  return self->status.cap_val;
}
