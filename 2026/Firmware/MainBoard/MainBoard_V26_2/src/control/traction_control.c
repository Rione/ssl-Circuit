#include "traction_control.h"

#include <math.h>
#include <string.h>

#include "mymath.h"

void TCS_Init(TractionControl* self) {
  memset(self, 0, sizeof(TractionControl));

  // デフォルトパラメータのロード
  self->config.max_accel = TCS_MAX_ACCEL;
  self->config.max_jerk = TCS_MAX_JERK;
  self->config.max_ang_accel = TCS_MAX_ANG_ACCEL;
  self->config.max_ang_jerk = TCS_MAX_ANG_JERK;

  self->config.geom_k = TCS_GEOM_K;
  self->config.geom_slip_thresh = TCS_GEOM_SLIP_THRESH;
  self->config.rot_slip_thresh = TCS_ROT_SLIP_THRESH;

  self->config.slip_gain = TCS_SLIP_GAIN;
  self->config.min_gain = TCS_MIN_GAIN;
  self->config.recovery_rate = TCS_RECOVERY_RATE;
  self->config.deadzone_speed = TCS_DEADZONE_SPEED_MPS;
  self->config.nominal_voltage = TCS_NOMINAL_VOLTAGE;

  self->config.enable_tcs = (TCS_ENABLE != 0);
  self->config.enable_s_curve = (TCS_ENABLE_S_CURVE != 0);

  TCS_Reset(self);
}

void TCS_Reset(TractionControl* self) {
  self->current_vx = 0.0f;
  self->current_vy = 0.0f;
  self->current_omega = 0.0f;

  self->current_ax = 0.0f;
  self->current_ay = 0.0f;
  self->current_alpha = 0.0f;

  self->geom_residual = 0.0f;
  self->rot_residual = 0.0f;
  self->is_slipping = false;
  self->trans_gain = 1.0f;
}

void TCS_SmoothVelocity(TractionControl* self, float target_vx, float target_vy,
                        float target_omega, float* out_vx, float* out_vy,
                        float* out_omega, float dt) {
  if (!self->config.enable_s_curve || dt <= 0.0f) {
    self->current_vx = target_vx;
    self->current_vy = target_vy;
    self->current_omega = target_omega;
    self->current_ax = 0.0f;
    self->current_ay = 0.0f;
    self->current_alpha = 0.0f;
    *out_vx = target_vx;
    *out_vy = target_vy;
    *out_omega = target_omega;
    return;
  }

  // 1. 並進速度ベクトル (vx, vy) の S字平滑化 (方向を保つ2Dベクトル制限)
  float dv_x = target_vx - self->current_vx;
  float dv_y = target_vy - self->current_vy;
  float dv_norm = sqrtf(dv_x * dv_x + dv_y * dv_y);

  if (dv_norm < 1e-4f) {
    self->current_vx = target_vx;
    self->current_vy = target_vy;
    self->current_ax = 0.0f;
    self->current_ay = 0.0f;
  } else {
    float des_ax = dv_x / dt;
    float des_ay = dv_y / dt;
    float des_a_norm = sqrtf(des_ax * des_ax + des_ay * des_ay);

    if (des_a_norm > self->config.max_accel) {
      float a_scale = self->config.max_accel / des_a_norm;
      des_ax *= a_scale;
      des_ay *= a_scale;
    }

    float da_x = des_ax - self->current_ax;
    float da_y = des_ay - self->current_ay;
    float da_norm = sqrtf(da_x * da_x + da_y * da_y);
    float max_da = self->config.max_jerk * dt;

    if (da_norm > max_da) {
      float j_scale = max_da / da_norm;
      da_x *= j_scale;
      da_y *= j_scale;
    }

    self->current_ax += da_x;
    self->current_ay += da_y;

    self->current_vx += self->current_ax * dt;
    self->current_vy += self->current_ay * dt;

    float new_dv_x = target_vx - self->current_vx;
    float new_dv_y = target_vy - self->current_vy;
    if ((dv_x * new_dv_x + dv_y * new_dv_y) <= 0.0f) {
      self->current_vx = target_vx;
      self->current_vy = target_vy;
      self->current_ax = 0.0f;
      self->current_ay = 0.0f;
    }
  }

  // 2. 回転角速度 omega の S字平滑化 (1D スカラー制限)
  float d_omega = target_omega - self->current_omega;
  if (fabsf(d_omega) < 1e-4f) {
    self->current_omega = target_omega;
    self->current_alpha = 0.0f;
  } else {
    float des_alpha = d_omega / dt;
    des_alpha = Constrain(des_alpha, -self->config.max_ang_accel,
                          self->config.max_ang_accel);

    float d_alpha = des_alpha - self->current_alpha;
    float max_d_alpha = self->config.max_ang_jerk * dt;
    d_alpha = Constrain(d_alpha, -max_d_alpha, max_d_alpha);

    self->current_alpha += d_alpha;
    self->current_omega += self->current_alpha * dt;

    float new_d_omega = target_omega - self->current_omega;
    if ((d_omega * new_d_omega) <= 0.0f) {
      self->current_omega = target_omega;
      self->current_alpha = 0.0f;
    }
  }

  *out_vx = self->current_vx;
  *out_vy = self->current_vy;
  *out_omega = self->current_omega;
}


void TCS_DetectSlip(TractionControl* self, const float actual_wheel_vel[4],
                    float gyro_yaw_rate, float current_speed_mps, float dt) {
  (void)dt;
  if (!self->config.enable_tcs) {
    self->is_slipping = false;
    self->trans_gain = 1.0f;
    return;
  }

  // 低速デッドゾーン: 極低速域ではエンコーダの量子化誤差や立ち上がり遅れによる
  // 誤判定を防ぐため、スリップ介入をバイパスする
  if (fabsf(current_speed_mps) < self->config.deadzone_speed) {
    self->is_slipping = false;
    return;
  }

  float k = self->config.geom_k;

  // 1. 4輪オムニホイール幾何学的残差の計算 (重心位置に非依存)
  // 拘束式: w0 - k*w1 + k*w2 - w3 = 0 (正常時)
  self->geom_residual = actual_wheel_vel[0] - k * actual_wheel_vel[1] +
                        k * actual_wheel_vel[2] - actual_wheel_vel[3];

  // 2. 旋回ジャイロ残差の計算 (オドメトリ角速度 - IMU角速度)
  float v_sum = 0.0f;
  for (int i = 0; i < 4; i++) {
    v_sum += actual_wheel_vel[i] * ROBOT_WHEEL_RADIUS;
  }
  float odom_yaw_rate = v_sum / (4.0f * ROBOT_WHEEL_BASE_RADIUS);
  self->rot_residual = odom_yaw_rate - gyro_yaw_rate;

  // 3. スリップ判定 (幾何残差または旋回ジャイロ残差が閾値を超過)
  bool geom_slip = fabsf(self->geom_residual) > self->config.geom_slip_thresh;
  bool rot_slip = fabsf(self->rot_residual) > self->config.rot_slip_thresh;

  self->is_slipping = geom_slip || rot_slip;
}

void TCS_ApplyIntervention(TractionControl* self, float* vx, float* vy,
                           float dt) {
  if (!self->config.enable_tcs || dt <= 0.0f) {
    return;
  }

  if (self->is_slipping) {
    // 幾何残差ベースの超過率
    float geom_excess = (fabsf(self->geom_residual) - self->config.geom_slip_thresh) /
                        self->config.geom_slip_thresh;
    if (geom_excess < 0.0f) geom_excess = 0.0f;

    // 旋回残差ベースの超過率
    float rot_excess = (fabsf(self->rot_residual) - self->config.rot_slip_thresh) /
                       self->config.rot_slip_thresh;
    if (rot_excess < 0.0f) rot_excess = 0.0f;

    float total_excess = (geom_excess > rot_excess) ? geom_excess : rot_excess;

    // 目標抑制ゲインの計算 (下限は min_gain = 0.70 などで失速を防止)
    float target_gain = 1.0f - self->config.slip_gain * total_excess;
    target_gain = Constrain(target_gain, self->config.min_gain, 1.0f);

    // スリップ検知時は瞬時に抑制 (即時介入)
    if (target_gain < self->trans_gain) {
      self->trans_gain = target_gain;
    }
  } else {
    // グリップ回復時はランプ関数で素早く滑らかに復帰 (recovery_rate = 6.0/s)
    if (self->trans_gain < 1.0f) {
      self->trans_gain += self->config.recovery_rate * dt;
      if (self->trans_gain > 1.0f) {
        self->trans_gain = 1.0f;
      }
    }
  }

  // ★ 車輪個別ではなく、機体並進ベクトル (vx, vy) を一括等比スケーリング！
  // 4輪の推力比率を 100% 維持するため、推力を抑制してもロボットの姿勢や進行方向は一切乱れない
  *vx *= self->trans_gain;
  *vy *= self->trans_gain;
}

void TCS_CompensateVoltage(const TractionControl* self, float wheel_vel[4],
                           float battery_voltage) {
  if (battery_voltage < 10.0f || battery_voltage > 25.0f) {
    return;
  }

  float v_ratio = self->config.nominal_voltage / battery_voltage;
  float scale = Constrain(v_ratio, 1.0f, 1.25f);

  for (int i = 0; i < 4; i++) {
    wheel_vel[i] *= scale;
  }
}
