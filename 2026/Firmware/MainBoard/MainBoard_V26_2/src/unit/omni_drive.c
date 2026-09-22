#include "omni_drive.h"

#include <math.h>
#include <stdio.h>

void OmniDrive_Init(OmniDrive* self, Serial* serials) {
  for (int i = 0; i < 4; i++) {
    self->serials[i] = &serials[i];
  }
  self->emg = 0;
  self->ready = 0;
  for (int i = 0; i < 4; i++) {
    MAF_Init(&self->maf[i], 25);
  }
  TCS_Init(&self->tcs);
}

void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
  OmniDrive_SetVelEx(self, vel_x, vel_y, vel_angle, 0.0f, 0.0f);
}

void OmniDrive_SetVelEx(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle,
                        float gyro_yaw_rate, float battery_voltage) {
  float vx_m = vel_x / 1000.0f;
  float vy_m = vel_y / 1000.0f;
  float omega_rad = vel_angle * 0.001f;
  const float dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;

  // 1. S字加減速 (ジャーク・加速度制限) による速度平滑化
  float smooth_vx = vx_m;
  float smooth_vy = vy_m;
  float smooth_omega = omega_rad;
  TCS_SmoothVelocity(&self->tcs, vx_m, vy_m, omega_rad, &smooth_vx, &smooth_vy,
                     &smooth_omega, dt);

  // 2. スリップ検知 (幾何学的残差拘束 & IMU旋回ジャイロ照合)
  float current_speed_mps = sqrtf(smooth_vx * smooth_vx + smooth_vy * smooth_vy);
  TCS_DetectSlip(&self->tcs, self->vel_wheel_angular, gyro_yaw_rate, current_speed_mps, dt);

  // 3. 能動トラクション介入 (並進ベクトル一括等比スケーリング)
  // ★ 車輪個別ではなく並進ベクトルを一括でスケールダウンするため、
  // 4輪の推力比率を 100% 維持し、姿勢や進行方向の崩れを完全に防ぐ！
  TCS_ApplyIntervention(&self->tcs, &smooth_vx, &smooth_vy, dt);

  // 4. オムニホイール逆運動学: v_w = -vx*sin(θ) + vy*cos(θ) + R*ω
  // ※ smooth_omega (姿勢制御) はスケーリングせず維持されるため、ヘディングロックが確実に機能する
  float target_wheel_angular[4];
  for (int i = 0; i < 4; i++) {
    float v_wheel_linear =
        -smooth_vx * SinDeg(ROBOT_MOTOR_DEGREE[i]) +
        smooth_vy * CosDeg(ROBOT_MOTOR_DEGREE[i]) +
        ROBOT_WHEEL_BASE_RADIUS * smooth_omega;

    target_wheel_angular[i] = v_wheel_linear / ROBOT_WHEEL_RADIUS;
  }

  // 5. 電圧変動補正 (バッテリー低下時のトルク抜け補償)
  if (battery_voltage > 0.0f) {
    TCS_CompensateVoltage(&self->tcs, target_wheel_angular, battery_voltage);
  }

  // 6. 出力整形式・最大角速度クランプ
  int16_t m[4];
  for (int i = 0; i < 4; i++) {
    float v_clamped = Constrain(target_wheel_angular[i], -100.0f, 100.0f);
    int16_t raw_m = (int16_t)(v_clamped * 100.0f);

    // S字制限が無効な場合のみ旧MAFフィルタを適用
    if (!self->tcs.config.enable_s_curve) {
      raw_m = MAF_Update(&self->maf[i], raw_m);
    }
    m[i] = raw_m;
  }

  OmniDrive_Send(self, m, 1);  // command: 1 (Drive)
}

void OmniDrive_SetFree(OmniDrive* self) {
  TCS_Reset(&self->tcs);
  int16_t m[4] = {0, 0, 0, 0};
  OmniDrive_Send(self, m, 0);  // command: 0 (Free)
}

void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command) {
  static Timer timer = {0};

  // 1ms 経過するまで送信しない
  if (Timer_ReadMs(&timer) < 1) return;
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
