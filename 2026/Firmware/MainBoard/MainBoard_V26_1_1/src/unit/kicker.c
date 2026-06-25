#include "kicker.h"

void Kicker_Init(Kicker* self, CanBus* can) {
  self->can = can;
  self->status.do_direct_status = 0;
  self->status.cap_val = 0;
}

void Kicker_Kick(Kicker* self, uint8_t is_straight, uint8_t power, uint8_t do_direct) {
  static Timer timer = {0};
  if (Timer_ReadMs(&timer) < ROBOT_KICK_INTERVAL_MS) return;

  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {power, (uint8_t)(do_direct ? 0xFF : 0), 0, 0, 0, 0, 0, 0},
  };
  Can_Send(self->can, &can_data);

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

void Kicker_CancelDirect(Kicker* self, uint8_t is_straight) {
  CanData can_data = {
      .stdId = is_straight ? CAN_ID_TX_STRAIGHT_KICK : CAN_ID_TX_CHIP_KICK,
      .data = {0},
  };
  Can_Send(self->can, &can_data);
}

uint16_t Kicker_GetCapVal(Kicker* self) {
  return self->status.cap_val;
}
