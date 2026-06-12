#include "omni_drive.h"

#include <stdio.h>

void OmniDrive_Init(OmniDrive* self, Serial* serials) {
  for (int i = 0; i < 4; i++) {
    self->serials[i] = &serials[i];
  }
  self->emg = 0;
  self->ready = 0;
}

void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
  float vx_m = vel_x / 1000.0f;
  float vy_m = vel_y / 1000.0f;
  int16_t m[4];

  for (int i = 0; i < 4; i++) {
    // オムニホイール逆運動学: v_w = -vx*sin(θ) + vy*cos(θ) + R*ω
    float v_wheel_linear =
        -vx_m * SinDeg(ROBOT_MOTOR_DEGREE[i]) +
        vy_m * CosDeg(ROBOT_MOTOR_DEGREE[i]) +
        ROBOT_WHEEL_BASE_RADIUS * vel_angle;

    // タイヤの角速度[rad/s]に変換
    float v_wheel_angular = v_wheel_linear / ROBOT_WHEEL_RADIUS;
    m[i] = (int16_t)v_wheel_angular * 100;
  }

  OmniDrive_Send(self, m, 1);  // command: 1 (Drive)
}

void OmniDrive_SetFree(OmniDrive* self) {
  int16_t m[4] = {0, 0, 0, 0};
  OmniDrive_Send(self, m, 0);  // command: 0 (Free)
}

void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command) {
  static uint8_t send_data[11];
  send_data[0] = 0xFF;
  send_data[1] = command;
  send_data[2] = (uint8_t)((m[0] >> 8) & 0xFF);
  send_data[3] = (uint8_t)(m[0] & 0xFF);
  send_data[4] = (uint8_t)((m[1] >> 8) & 0xFF);
  send_data[5] = (uint8_t)(m[1] & 0xFF);
  send_data[6] = (uint8_t)((m[2] >> 8) & 0xFF);
  send_data[7] = (uint8_t)(m[2] & 0xFF);
  send_data[8] = (uint8_t)((m[3] >> 8) & 0xFF);
  send_data[9] = (uint8_t)(m[3] & 0xFF);
  send_data[10] = 0xAA;

  Serial_Write(self->serials[0], send_data, 11);
}

void OmniDrive_Recv(OmniDrive* self) {
  static uint8_t recv_data[4][3];
  static uint8_t index[4] = {0};

  for (int i = 0; i < 4; i++) {
    if (!Serial_Available(self->serials[i])) continue;

    uint8_t recv_byte = Serial_Read(self->serials[i]);

    if (index[i] == 0) {
      if (recv_byte == 0xFF) {
        index[i]++;
      }
    } else if (index[i] == 4) {
      if (recv_byte == 0xAA) {
        self->emg = recv_data[i][0] & 0x01;
        self->ready = (recv_data[i][0] >> 1) & 0x01;
        self->vel_wheel_angular[i] =
            (int16_t)((recv_data[i][1] << 8) | recv_data[i][2]);
      }
      index[i] = 0;
    } else {
      recv_data[i][index[i] - 1] = recv_byte;
      index[i]++;
    }
  }
}

void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
  float v_wheel_linear[4];
  for (int i = 0; i < 4; i++) {
    v_wheel_linear[i] = self->vel_wheel_angular[i] * ROBOT_WHEEL_RADIUS;
  }

  float vx_sum = 0.0f, vy_sum = 0.0f, v_sum = 0.0f;
  for (int i = 0; i < 4; i++) {
    vx_sum += v_wheel_linear[i] * (-SinDeg(ROBOT_MOTOR_DEGREE[i]));
    vy_sum += v_wheel_linear[i] * CosDeg(ROBOT_MOTOR_DEGREE[i]);
    v_sum += v_wheel_linear[i];
  }

  *vel_x = (int16_t)(vx_sum / 2.0f * 1000.0f);
  *vel_y = (int16_t)(vy_sum / 2.0f * 1000.0f);
  *vel_angle = (int16_t)(v_sum / (4.0f * ROBOT_WHEEL_BASE_RADIUS));
}
