#include "ui.h"

#include "robot.h"

void UI_Init(UI* self, Serial* serial) {
  self->serial = serial;
}

void UI_Recv(UI* self, UiStatus* status) {
  if (!Serial_Available(self->serial)) return;

  static uint8_t recv_data[2];
  static uint8_t index = 0;
  uint8_t recv_byte = Serial_Read(self->serial);

  if (index == 0) {
    if (recv_byte == 0xFF) {
      index++;
    }
  } else if (index == 3) {
    if (recv_byte == 0xAA) {
      status->flags = recv_data[0];
      status->power_data = recv_data[1];
    }
    index = 0;
  } else {
    recv_data[index - 1] = recv_byte;
    index++;
  }
}

void UI_Send(UI* self, const struct Robot* robot) {
  uint8_t send_data[4];
  send_data[0] = 0xFF;
  send_data[1] = robot->info.battery_voltage;
  send_data[2] = robot->info.kicker_status.cap_val;
  send_data[3] = 0xAA;

  Serial_Write(self->serial, send_data, sizeof(send_data));
}
