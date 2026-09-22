#include "omni_drive.h"

#include <string.h>

#include <stdio.h>

// v_i = A_i*vx + B_i*vy + C*omega (A_i=-sin(θi), B_i=cos(θi), C=ROBOT_WHEEL_BASE_RADIUS)
// というホイール正運動学モデルの最小二乗解(擬似逆行列)を計算し、
// OmniDrive_GetVelで使う係数(ik_vx/ik_vy/ik_omega)としてキャッシュする。
// ROBOT_MOTOR_DEGREEが対称配置([θ,180-θ,-(180-θ),-θ]相当)でない場合、
// vx/vy/omegaは単純な固定係数では正しく分離できず3x3の連立方程式を解く必要があるため、
// 起動時に一般解として計算しておく。
static void OmniDrive_ComputeInverseKinematics(OmniDrive* self) {
  float a[4], b[4];
  const float c = ROBOT_WHEEL_BASE_RADIUS;
  for (int i = 0; i < 4; i++) {
    a[i] = -SinDeg(ROBOT_MOTOR_DEGREE[i]);
    b[i] = CosDeg(ROBOT_MOTOR_DEGREE[i]);
  }

  // M^T M (3x3, M の各行が [a_i, b_i, c])
  float m00 = 0.0f, m01 = 0.0f, m02 = 0.0f, m11 = 0.0f, m12 = 0.0f;
  for (int i = 0; i < 4; i++) {
    m00 += a[i] * a[i];
    m01 += a[i] * b[i];
    m02 += a[i] * c;
    m11 += b[i] * b[i];
    m12 += b[i] * c;
  }
  float m22 = 4.0f * c * c;
  float m[3][3] = {
      {m00, m01, m02},
      {m01, m11, m12},
      {m02, m12, m22},
  };

  float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
              m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
              m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  float inv[3][3];
  if (Abs(det) < 1e-9f) {
    // ホイール配置が特異(理論上あり得ないはず)の場合は係数0でフォールバックする
    memset(inv, 0, sizeof(inv));
  } else {
    float inv_det = 1.0f / det;
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
  }

  // Pinv (3x4) = inv(M^T M) * M^T
  for (int i = 0; i < 4; i++) {
    self->ik_vx[i] = inv[0][0] * a[i] + inv[0][1] * b[i] + inv[0][2] * c;
    self->ik_vy[i] = inv[1][0] * a[i] + inv[1][1] * b[i] + inv[1][2] * c;
    self->ik_omega[i] = inv[2][0] * a[i] + inv[2][1] * b[i] + inv[2][2] * c;
  }
}

void OmniDrive_Init(OmniDrive* self, Serial* serials) {
  for (int i = 0; i < 4; i++) {
    self->serials[i] = &serials[i];
  }
  self->emg = 0;
  self->ready = 0;
  for (int i = 0; i < 4; i++) {
    MAF_Init(&self->maf[i], 25);
  }
  OmniDrive_ComputeInverseKinematics(self);
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
        ROBOT_WHEEL_BASE_RADIUS * vel_angle * 0.001f;

    // タイヤの角速度[rad/s]に変換し、最大角速度で制限
    float v_wheel_angular = v_wheel_linear / ROBOT_WHEEL_RADIUS;
    v_wheel_angular = Constrain(v_wheel_angular, -100.0f, 100.0f);
    m[i] = (int16_t)(v_wheel_angular * 100);
    m[i] = MAF_Update(&self->maf[i], m[i]);
  }

  OmniDrive_Send(self, m, 1);  // command: 1 (Drive)
}

void OmniDrive_SetFree(OmniDrive* self) {
  int16_t m[4] = {0, 0, 0, 0};
  OmniDrive_Send(self, m, 0);  // command: 0 (Free)
}

void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command) {
  static Timer timer = {0};

  // 1ms 経過するまで送信しない
  if (Timer_ReadMs(&timer) < 1) return;

  // 前回のDMA送信が完了していない場合は送信できないため、タイマーを進めず
  // 次ループで再送を試みる
  if (self->serials[2]->huart->gState == HAL_UART_STATE_BUSY_TX) return;
  Timer_Reset(&timer);

  static uint8_t send_data[11];
  send_data[0] = 0xAA;
  send_data[1] = command;
  send_data[2] = (uint8_t)((m[0] >> 8) & 0xFF);
  send_data[3] = (uint8_t)(m[0] & 0xFF);
  send_data[4] = (uint8_t)((m[1] >> 8) & 0xFF);
  send_data[5] = (uint8_t)(m[1] & 0xFF);
  send_data[6] = (uint8_t)((m[2] >> 8) & 0xFF);
  send_data[7] = (uint8_t)(m[2] & 0xFF);
  send_data[8] = (uint8_t)((m[3] >> 8) & 0xFF);
  send_data[9] = (uint8_t)(m[3] & 0xFF);
  send_data[10] = 0xFF;

  Serial_Write(self->serials[2], send_data, 11);
}

void OmniDrive_Recv(OmniDrive* self) {
  static uint8_t recv_data[4][3];
  static uint8_t index[4] = {0};

  for (int i = 0; i < 4; i++) {
    while (Serial_Available(self->serials[i])) {
      uint8_t recv_byte = Serial_Read(self->serials[i]);

      if (index[i] == 0) {
        if (recv_byte == 0xFF) {
          index[i]++;
        } else {
          index[i] = 0;
        }
      } else if (index[i] == 4) {
        if (recv_byte == 0xAA) {
          self->emg = recv_data[i][0] & 0x01;
          self->ready = (recv_data[i][0] >> 1) & 0x01;
          self->vel_wheel_angular[i] =
              (int16_t)((recv_data[i][1] << 8) | recv_data[i][2]) * 0.01;
        }
        index[i] = 0;
      } else {
        recv_data[i][index[i] - 1] = recv_byte;
        index[i]++;
      }
    }
  }
}

void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
  float v_wheel_linear[4];
  for (int i = 0; i < 4; i++) {
    v_wheel_linear[i] = self->vel_wheel_angular[i] * ROBOT_WHEEL_RADIUS;
  }

  // OmniDrive_Initで計算済みの最小二乗擬似逆行列係数を用いて逆運動学を解く
  float vx = 0.0f, vy = 0.0f, omega = 0.0f;
  for (int i = 0; i < 4; i++) {
    vx += self->ik_vx[i] * v_wheel_linear[i];
    vy += self->ik_vy[i] * v_wheel_linear[i];
    omega += self->ik_omega[i] * v_wheel_linear[i];
  }

  *vel_x = (int16_t)(vx * 1000.0f);
  *vel_y = (int16_t)(vy * 1000.0f);
  *vel_angle = (int16_t)omega;
}
