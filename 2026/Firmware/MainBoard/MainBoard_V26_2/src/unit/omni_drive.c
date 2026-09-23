#include "omni_drive.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static void OmniDrive_MonitorRx(OmniDrive* self);

// 逆運動学 H (4x3, 行 [-sinθi, cosθi, R]) から最小二乗疑似逆行列 (HᵀH)⁻¹Hᵀ を求める。
// 55°/135° 配置は Σsin²θ≠Σcos²θ, Σcosθ≠0 のため、単純な Σ/2 では vx が約17%過大になる
static void OmniDrive_ComputeForwardKinematics(OmniDrive* self) {
  float h[4][3];
  for (int i = 0; i < 4; i++) {
    h[i][0] = -SinDeg(ROBOT_MOTOR_DEGREE[i]);
    h[i][1] = CosDeg(ROBOT_MOTOR_DEGREE[i]);
    h[i][2] = ROBOT_WHEEL_BASE_RADIUS;
  }

  float a[3][3] = {{0}};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      for (int i = 0; i < 4; i++) a[r][c] += h[i][r] * h[i][c];
    }
  }

  // 3x3 逆行列 (余因子展開)
  float inv[3][3];
  inv[0][0] = a[1][1] * a[2][2] - a[1][2] * a[2][1];
  inv[0][1] = a[0][2] * a[2][1] - a[0][1] * a[2][2];
  inv[0][2] = a[0][1] * a[1][2] - a[0][2] * a[1][1];
  inv[1][0] = a[1][2] * a[2][0] - a[1][0] * a[2][2];
  inv[1][1] = a[0][0] * a[2][2] - a[0][2] * a[2][0];
  inv[1][2] = a[0][2] * a[1][0] - a[0][0] * a[1][2];
  inv[2][0] = a[1][0] * a[2][1] - a[1][1] * a[2][0];
  inv[2][1] = a[0][1] * a[2][0] - a[0][0] * a[2][1];
  inv[2][2] = a[0][0] * a[1][1] - a[0][1] * a[1][0];
  float det = a[0][0] * inv[0][0] + a[0][1] * inv[1][0] + a[0][2] * inv[2][0];

  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < 4; i++) {
      float sum = 0.0f;
      for (int c = 0; c < 3; c++) sum += inv[r][c] * h[i][c];
      self->fk[r][i] = sum / det;
    }
  }
}

void OmniDrive_Init(OmniDrive* self, Serial* serials) {
  for (int i = 0; i < 4; i++) {
    self->serials[i] = &serials[i];
  }
  self->emg = 0;
  self->ready = 0;
  for (int i = 0; i < 4; i++) {
    self->vel_wheel_angular[i] = 0.0f;
    self->target_wheel_angular[i] = 0.0f;
    self->wheel_status[i] = 0;
    self->wheel_frame_count[i] = 0;
    self->wheel_rx_restart_count[i] = 0;
    self->wheel_last_frame_tick[i] = 0;
    self->wheel_rx_stalled[i] = false;
    MAF_Init(&self->maf[i], 25);
  }
  self->rx_monitor_started = false;
  OmniDrive_ComputeForwardKinematics(self);
  TCS_Init(&self->tcs);
}

void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
  OmniDrive_SetVelEx(self, vel_x, vel_y, vel_angle, NULL);
}

// 呼び出し間隔の実測dt [s]。制御ループは基本1ms周期だが、他処理のブロッキング
// (例: Rock5A SPIストール検知時のprintf)で稀に周期が乱れることがある。
// 固定値dtを使うとTCSの加速度差分計算(オドメトリ速度の微分)がその分だけ実際の
// 数百倍の値になり、スリップ誤検知や速度平滑化の破綻を招くため、実測して使う。
static float OmniDrive_MeasureDt(void) {
  static Timer timer = {0};
  static bool initialized = false;
  const float kNominalDt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;
  const float kMaxDt = 0.05f;  // 50ms超のギャップはクランプ (異常時の暴走防止)

  if (!initialized) {
    Timer_Init(&timer);
    Timer_Reset(&timer);
    initialized = true;
    return kNominalDt;
  }

  float dt = Timer_Read(&timer);
  Timer_Reset(&timer);
  if (dt <= 0.0f) dt = kNominalDt;
  if (dt > kMaxDt) dt = kMaxDt;
  return dt;
}

void OmniDrive_SetVelEx(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle,
                        const Imu* imu) {
  float vx_m = vel_x / 1000.0f;
  float vy_m = vel_y / 1000.0f;
  float omega_rad = vel_angle * 0.001f;
  const float dt = OmniDrive_MeasureDt();

  // 1. TCS: 対地速度推定 → スリップ検知 → スリップ量に応じたS字加減速
  TCSInput in;
  for (int i = 0; i < 4; i++) {
    in.wheel_vel[i] = self->vel_wheel_angular[i];
  }
  OmniDrive_GetVelF(self, &in.odom_vx, &in.odom_vy, &in.odom_omega);
  in.has_imu = (imu != NULL);
  in.gyro_yaw_rate = in.has_imu ? imu->yaw_rate : 0.0f;
  in.accel_x = in.has_imu ? imu->accel_robot_x : 0.0f;
  in.accel_y = in.has_imu ? imu->accel_robot_y : 0.0f;

  float cmd_vx, cmd_vy, cmd_omega;
  TCS_Update(&self->tcs, &in, vx_m, vy_m, omega_rad, &cmd_vx, &cmd_vy, &cmd_omega, dt);

  // 2. オムニホイール逆運動学: v_w = -vx*sin(θ) + vy*cos(θ) + R*ω
  // ※ バッテリー電圧補正は WheelUnit 側 (FOC に電源電圧を渡している) で行われる。
  //    ここで目標角速度を増やすと速度閉ループのため実速度が指令より速くなってしまう
  float target_wheel_angular[4];
  for (int i = 0; i < 4; i++) {
    float v_wheel_linear =
        -cmd_vx * SinDeg(ROBOT_MOTOR_DEGREE[i]) +
        cmd_vy * CosDeg(ROBOT_MOTOR_DEGREE[i]) +
        ROBOT_WHEEL_BASE_RADIUS * cmd_omega;

    target_wheel_angular[i] = v_wheel_linear / ROBOT_WHEEL_RADIUS;
    self->target_wheel_angular[i] = target_wheel_angular[i];  // ログ用 (クランプ前)
  }

  // 3. 出力整形・最大角速度クランプ
  int16_t m[4];
  for (int i = 0; i < 4; i++) {
    float v_clamped = Constrain(target_wheel_angular[i], -100.0f, 100.0f);
    int16_t raw_m = (int16_t)(v_clamped * 100.0f);

    // MAFは常に更新しておき (切替直後に古い値が出ないように)、S字制限が無効な場合のみ使う
    int16_t maf_m = (int16_t)MAF_Update(&self->maf[i], raw_m);
    m[i] = self->tcs.config.enable_s_curve ? raw_m : maf_m;
  }

  OmniDrive_Send(self, m, 1);  // command: 1 (Drive)
}

void OmniDrive_SetFree(OmniDrive* self) {
  TCS_Reset(&self->tcs);
  for (int i = 0; i < 4; i++) {
    self->target_wheel_angular[i] = 0.0f;
  }
  int16_t m[4] = {0, 0, 0, 0};
  OmniDrive_Send(self, m, 0);  // command: 0 (Free)
}

// 電圧モード (command 2): 各輪の印加電圧 [V] を送る。WheelUnit側で ±MAX_AMP_VOLT(5V) にクランプされ、
// 電源電圧での正規化もWheelUnit側で行われる。0V は空転ではなく短絡ブレーキになる。
// ※ WheelUnitのホイールロック検知は +5.0V ちょうどが1秒続くと出力を切るので、上限張り付きに注意
void OmniDrive_SetVoltage(OmniDrive* self, const float volt[4]) {
  int16_t m[4];
  for (int i = 0; i < 4; i++) {
    m[i] = (int16_t)(Constrain(volt[i], -5.0f, 5.0f) * 100.0f);
    self->target_wheel_angular[i] = 0.0f;
  }
  OmniDrive_Send(self, m, 2);  // command: 2 (Voltage)
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

// 受信した車輪角速度を受理するか判定する。前回値から OMNI_WHEEL_MAX_JUMP_RADPS を超えて
// 跳んだ値は保留し、次のフレームも保留値の近くなら本物の変化として受理する
// (ヘッダ/フッタ判定だけではデータ中の0xFF/0xAAでフレームがずれた誤値を受理しうるため)
static bool OmniDrive_AcceptWheelVel(float prev, float value, float* pending,
                                     bool* has_pending) {
  if (fabsf(value - prev) <= OMNI_WHEEL_MAX_JUMP_RADPS) {
    *has_pending = false;
    return true;
  }
  if (*has_pending && fabsf(value - *pending) <= OMNI_WHEEL_MAX_JUMP_RADPS) {
    *has_pending = false;
    return true;
  }
  *pending = value;
  *has_pending = true;
  return false;
}

void OmniDrive_Recv(OmniDrive* self) {
  static uint8_t recv_data[4][3];
  static uint8_t index[4] = {0};
  static float pending[4];
  static bool has_pending[4] = {false};

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
          self->wheel_status[i] = recv_data[i][0];
          self->wheel_frame_count[i]++;
          self->wheel_last_frame_tick[i] = HAL_GetTick();
          self->wheel_rx_stalled[i] = false;
          float vel = (int16_t)((recv_data[i][1] << 8) | recv_data[i][2]) * 0.01f;
          if (OmniDrive_AcceptWheelVel(self->vel_wheel_angular[i], vel, &pending[i],
                                       &has_pending[i])) {
            self->vel_wheel_angular[i] = vel;
          }
        }
        index[i] = 0;
      } else {
        recv_data[i][index[i] - 1] = recv_byte;
        index[i]++;
      }
    }
  }

  OmniDrive_MonitorRx(self);
}

// 受信監視: 最後の正常フレームから OMNI_RX_TIMEOUT_MS 以上たった輪は受信を再開する。
// 実機では、MainBoard の動作中に WheelUnit が起動 (24Vの投入・瞬断) すると、
// 受信エラーのコールバックも呼ばれないまま4輪の受信が止まり、車輪速度が固まった。
// 原因の特定用に、途絶えるたびに最初の1回だけ USART/DMA の状態を出力する
// (WheelUnitが電源オフの間は OMNI_RX_TIMEOUT_MS ごとに再開を繰り返す)
static void OmniDrive_MonitorRx(OmniDrive* self) {
  uint32_t now = HAL_GetTick();
  if (!self->rx_monitor_started) {
    // 起動処理 (IMUキャリブレーション等) の時間を途絶と誤判定しないよう、初回に時刻を揃える
    for (int i = 0; i < 4; i++) self->wheel_last_frame_tick[i] = now;
    self->rx_monitor_started = true;
    return;
  }

  for (int i = 0; i < 4; i++) {
    if (now - self->wheel_last_frame_tick[i] < OMNI_RX_TIMEOUT_MS) continue;

    UART_HandleTypeDef* huart = self->serials[i]->huart;
    if (!self->wheel_rx_stalled[i]) {
      self->wheel_rx_stalled[i] = true;
      printf("# rx_stall wheel=%d t=%lu SR=0x%04lX CR3=0x%04lX NDTR=%lu EN=%lu RxState=0x%02X Err=0x%02lX\n",
             i, (unsigned long)now, (unsigned long)huart->Instance->SR,
             (unsigned long)huart->Instance->CR3, (unsigned long)huart->hdmarx->Instance->NDTR,
             (unsigned long)(huart->hdmarx->Instance->CR & DMA_SxCR_EN),
             (unsigned)huart->RxState, (unsigned long)huart->ErrorCode);
    }
    Serial_RestartRx(self->serials[i]);
    self->wheel_rx_restart_count[i]++;
    self->wheel_last_frame_tick[i] = now;  // 次の再開は OMNI_RX_TIMEOUT_MS 後
  }
}

void OmniDrive_GetVelF(const OmniDrive* self, float* vx, float* vy, float* omega) {
  float out[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 4; i++) {
    float v_wheel_linear = self->vel_wheel_angular[i] * ROBOT_WHEEL_RADIUS;
    for (int r = 0; r < 3; r++) out[r] += self->fk[r][i] * v_wheel_linear;
  }
  *vx = out[0];
  *vy = out[1];
  *omega = out[2];
}

void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
  float vx, vy, omega;
  OmniDrive_GetVelF(self, &vx, &vy, &omega);
  *vel_x = (int16_t)(vx * 1000.0f);
  *vel_y = (int16_t)(vy * 1000.0f);
  *vel_angle = (int16_t)(omega * 1000.0f);
}
