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
  self->config.accel_slip_thresh = TCS_ACCEL_SLIP_THRESH;
  self->config.accel_detect_min = TCS_ACCEL_DETECT_MIN;
  self->config.vel_slip_exit = TCS_VEL_SLIP_EXIT;
  self->config.slip_hold_s = TCS_SLIP_HOLD_MS * 1e-3f;
  self->config.slip_vel_margin = TCS_SLIP_VEL_MARGIN;

  self->config.ground_vel_kc = TCS_GROUND_VEL_KC;
  self->config.accel_lpf_hz = TCS_ACCEL_LPF_HZ;
  self->config.max_inertial_s = TCS_MAX_INERTIAL_MS * 1e-3f;

  self->config.min_gain = TCS_MIN_GAIN;
  self->config.slip_gain_cut = TCS_SLIP_GAIN_CUT;
  self->config.recovery_rate = TCS_RECOVERY_RATE;
  self->config.deadzone_speed = TCS_DEADZONE_SPEED_MPS;

  self->config.enable_tcs = (TCS_ENABLE != 0);
  self->config.enable_s_curve = (TCS_ENABLE_S_CURVE != 0);

  TCS_Reset(self);
}

void TCS_Reset(TractionControl* self) {
  self->current_ax = 0.0f;
  self->current_ay = 0.0f;
  self->current_alpha = 0.0f;
  self->seed_pending = true;

  self->geom_residual = 0.0f;
  self->rot_residual = 0.0f;
  self->accel_residual = 0.0f;
  self->slip_flags = 0;
  self->is_slipping = false;
  self->prev_slipping = false;
  self->slip_time_s = 0.0f;
  self->clear_time_s = 0.0f;
  self->accel_gain = 1.0f;
}

// 平滑化状態・推定状態を実測値から初期化する (惰性走行中からの再指令やS字の再有効化で
// 0 や古い指令値から始めると、実速度とかけ離れた指令がステップ状に出てしまうため)
static void TCS_Seed(TractionControl* self, const TCSInput* in) {
  self->current_vx = in->odom_vx;
  self->current_vy = in->odom_vy;
  self->current_omega = in->has_imu ? in->gyro_yaw_rate : in->odom_omega;
  self->current_ax = 0.0f;
  self->current_ay = 0.0f;
  self->current_alpha = 0.0f;

  self->odom_vx = in->odom_vx;
  self->odom_vy = in->odom_vy;
  self->ground_vx = in->odom_vx;
  self->ground_vy = in->odom_vy;
  self->a_odom_x = 0.0f;
  self->a_odom_y = 0.0f;
  self->a_imu_x = in->accel_x;
  self->a_imu_y = in->accel_y;

  self->seed_pending = false;
}

// 対地速度推定とスリップ検知
static void TCS_Estimate(TractionControl* self, const TCSInput* in, float dt) {
  const TCSConfig* cfg = &self->config;

  // 1. 加速度の算出 (オドメトリ微分とIMUに同じ一次LPFを掛けて位相を揃える)
  float tau = 1.0f / (2.0f * (float)PI * cfg->accel_lpf_hz);
  float alpha = dt / (tau + dt);
  float a_odom_raw_x = (in->odom_vx - self->odom_vx) / dt;
  float a_odom_raw_y = (in->odom_vy - self->odom_vy) / dt;
  self->odom_vx = in->odom_vx;
  self->odom_vy = in->odom_vy;
  self->a_odom_x += alpha * (a_odom_raw_x - self->a_odom_x);
  self->a_odom_y += alpha * (a_odom_raw_y - self->a_odom_y);
  self->a_imu_x += alpha * (in->accel_x - self->a_imu_x);
  self->a_imu_y += alpha * (in->accel_y - self->a_imu_y);

  // 2. 対地速度推定: 機体座標系での v' = a - ω×v を積分し、
  //    グリップ中のみオドメトリへ相補補正する (スリップ中は車輪速度を信用しない)
  float w = in->gyro_yaw_rate;
  float gvx = self->ground_vx;
  float gvy = self->ground_vy;
  self->ground_vx = gvx + (in->accel_x + w * gvy) * dt;
  self->ground_vy = gvy + (in->accel_y - w * gvx) * dt;
  if (!self->is_slipping || self->slip_time_s > cfg->max_inertial_s) {
    float kc = cfg->ground_vel_kc * dt;
    self->ground_vx += kc * (in->odom_vx - self->ground_vx);
    self->ground_vy += kc * (in->odom_vy - self->ground_vy);
  }

  // 3. 残差の計算
  // 4輪オムニホイール幾何学的残差 (重心位置に非依存)
  // 拘束式: w0 - k*w1 + k*w2 - w3 = 0 (正常時)
  float k = cfg->geom_k;
  self->geom_residual = in->wheel_vel[0] - k * in->wheel_vel[1] +
                        k * in->wheel_vel[2] - in->wheel_vel[3];
  // 旋回残差 (オドメトリ角速度 - IMU角速度)
  self->rot_residual = in->odom_omega - in->gyro_yaw_rate;
  // 加速度残差 (全輪が比例して空転する直進スリップは上2つに現れないため、これで検出する)
  // 指令加速度の方向に「車輪の方が車体より速く加減速している」成分だけを見る
  // (加速時の空転も減速時のロックもこの成分が正になる)。巡航中は車輪速度PIDの
  // リップルで a_odom が±数m/s²揺れ、ノルムで判定すると常時誤検知していたため、
  // S字が実際に加減速している間だけ判定する
  float cmd_a = sqrtf(self->current_ax * self->current_ax +
                      self->current_ay * self->current_ay);
  bool accelerating = cmd_a >= cfg->accel_detect_min;
  float ux = accelerating ? self->current_ax / cmd_a : 0.0f;
  float uy = accelerating ? self->current_ay / cmd_a : 0.0f;
  float dax = self->a_odom_x - self->a_imu_x;
  float day = self->a_odom_y - self->a_imu_y;
  self->accel_residual = dax * ux + day * uy;

  // 4. スリップ判定
  // 低速デッドゾーン: 指令・実測とも極低速ならエンコーダ量子化誤差による誤判定を避ける
  float cmd_speed = sqrtf(self->current_vx * self->current_vx +
                          self->current_vy * self->current_vy);
  float odom_speed = sqrtf(in->odom_vx * in->odom_vx + in->odom_vy * in->odom_vy);
  uint8_t flags = 0;
  if (cmd_speed >= cfg->deadzone_speed || odom_speed >= cfg->deadzone_speed) {
    if (fabsf(self->geom_residual) > cfg->geom_slip_thresh) flags |= TCS_SLIP_GEOM;
    if (fabsf(self->rot_residual) > cfg->rot_slip_thresh) flags |= TCS_SLIP_ROT;
    if (self->accel_residual > cfg->accel_slip_thresh) flags |= TCS_SLIP_ACCEL;
  }
  self->slip_flags = flags;

  if (flags) {
    self->is_slipping = true;
    self->clear_time_s = 0.0f;
  } else if (self->is_slipping) {
    // 解除: オドメトリと対地速度推定が一致する、または要因が消えてから一定時間経過。
    // 対地速度推定には0.1〜0.2m/s程度の定常的なずれが残ることがあり、
    // 一致だけを条件にすると解除されずに accel_gain が下限に張り付いていた
    self->clear_time_s += dt;
    float evx = in->odom_vx - self->ground_vx;
    float evy = in->odom_vy - self->ground_vy;
    if (sqrtf(evx * evx + evy * evy) < cfg->vel_slip_exit ||
        self->clear_time_s >= cfg->slip_hold_s) {
      self->is_slipping = false;
    }
  }
  self->slip_time_s = self->is_slipping ? self->slip_time_s + dt : 0.0f;

  // 5. 加速度上限比率の更新 (スリップ突入で乗算的に下げ、解除後は一定速度で戻す)
  //  - 下げるのはS字が加減速している間に突入したときだけ。巡航中の検知(後退巡航では
  //    幾何残差が定常的に閾値を超える)で下げると、IMU加速度≈0から目標倍率が即座に
  //    下限になり、次の反転を4.5m/s²程度でしか減速できなくなっていた
  //  - 下げるのは突入時の1回だけで、1回あたり最大 slip_gain_cut 倍まで。検知時点の
  //    IMU加速度はLPFやサンプリングの遅れで小さく見えるため、それだけを目標にすると
  //    減速開始のたびに下限まで落ちていた。本当に路面限界を超えていれば
  //    突入→カット→解除→回復→再突入を繰り返し、持続できる加速度に収束する
  bool slip_entered = self->is_slipping && !self->prev_slipping;
  self->prev_slipping = self->is_slipping;
  if (!cfg->enable_tcs) {
    self->accel_gain = 1.0f;
  } else if (slip_entered && accelerating) {
    // 実際に出ている加速度(指令方向成分)の9割を下限の目安にする
    float a_imu_along = fmaxf(0.0f, self->a_imu_x * ux + self->a_imu_y * uy);
    float target_gain = fmaxf(cfg->slip_gain_cut * self->accel_gain,
                              0.9f * a_imu_along / cfg->max_accel);
    target_gain = Constrain(target_gain, cfg->min_gain, 1.0f);
    if (target_gain < self->accel_gain) self->accel_gain = target_gain;
  } else if (!self->is_slipping && self->accel_gain < 1.0f) {
    self->accel_gain += cfg->recovery_rate * dt;
    if (self->accel_gain > 1.0f) self->accel_gain = 1.0f;
  }
}

// 並進速度ベクトル (vx, vy) の S字平滑化 (方向を保つ2Dベクトル制限)
static void TCS_SmoothTranslation(TractionControl* self, float target_vx,
                                  float target_vy, float max_accel, float dt) {
  float dv_x = target_vx - self->current_vx;
  float dv_y = target_vy - self->current_vy;
  float dv_norm = sqrtf(dv_x * dv_x + dv_y * dv_y);

  if (dv_norm < 1e-4f) {
    self->current_vx = target_vx;
    self->current_vy = target_vy;
    self->current_ax = 0.0f;
    self->current_ay = 0.0f;
    return;
  }

  // 目標到達時に加速度がジャーク制限内で0へ戻れるよう sqrt(2*j*|dv|) で頭打ちにする
  float a_norm = dv_norm / dt;
  float a_limit = fminf(max_accel, sqrtf(2.0f * self->config.max_jerk * dv_norm));
  if (a_norm > a_limit) a_norm = a_limit;
  float des_ax = dv_x / dv_norm * a_norm;
  float des_ay = dv_y / dv_norm * a_norm;

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

// 回転角速度 omega の S字平滑化 (1D スカラー制限)
static void TCS_SmoothRotation(TractionControl* self, float target_omega, float dt) {
  float d_omega = target_omega - self->current_omega;
  if (fabsf(d_omega) < 1e-4f) {
    self->current_omega = target_omega;
    self->current_alpha = 0.0f;
    return;
  }

  float alpha_limit = fminf(self->config.max_ang_accel,
                            sqrtf(2.0f * self->config.max_ang_jerk * fabsf(d_omega)));
  float des_alpha = Constrain(d_omega / dt, -alpha_limit, alpha_limit);

  float max_d_alpha = self->config.max_ang_jerk * dt;
  float d_alpha = Constrain(des_alpha - self->current_alpha, -max_d_alpha, max_d_alpha);

  self->current_alpha += d_alpha;
  self->current_omega += self->current_alpha * dt;

  float new_d_omega = target_omega - self->current_omega;
  if ((d_omega * new_d_omega) <= 0.0f) {
    self->current_omega = target_omega;
    self->current_alpha = 0.0f;
  }
}

// スリップ中は並進指令の大きさを 対地速度(指令方向成分) + slip_vel_margin 以下に抑える
// (ステップ状に切ると逆向きのトルクスパイクになるため、max_accel の速さで寄せる)
//  - 指令の向きは変えず大きさだけを削る。以前は指令ベクトルを対地速度推定ベクトルへ
//    2次元で引き寄せていたため、推定の横速度のずれ(実測で最大約1m/s)がそのまま横方向の
//    指令として注入され、機体が横に流れていた
//  - 車体が指令と同じ向きに進んでいるときの空転(車輪が車体より速い)だけを抑える。
//    減速・反転中に指令を対地速度へ寄せると、指令が押し戻されて減速しない→対地速度も
//    下がらない、の循環で反転が約1秒遅れ、行き過ぎていた
static void TCS_LimitToGround(TractionControl* self, float dt) {
  float speed = sqrtf(self->current_vx * self->current_vx +
                      self->current_vy * self->current_vy);
  if (speed < 1e-3f) return;
  float ux = self->current_vx / speed;
  float uy = self->current_vy / speed;

  float ground_along = self->ground_vx * ux + self->ground_vy * uy;
  if (ground_along <= 0.0f) return;  // 車体が逆向き/停止中 = 減速・反転中は引き戻さない

  float limit = ground_along + self->config.slip_vel_margin;
  if (speed <= limit) return;

  float new_speed = fmaxf(limit, speed - self->config.max_accel * dt);
  self->current_vx = ux * new_speed;
  self->current_vy = uy * new_speed;

  // 指令方向へ押し続ける加速度成分は捨てる (以後はジャーク制限付きで再加速する)
  float a_along = self->current_ax * ux + self->current_ay * uy;
  if (a_along > 0.0f) {
    self->current_ax -= a_along * ux;
    self->current_ay -= a_along * uy;
  }
}

void TCS_Update(TractionControl* self, const TCSInput* in, float target_vx,
                float target_vy, float target_omega, float* out_vx,
                float* out_vy, float* out_omega, float dt) {
  bool s_curve_enabled_now = self->config.enable_s_curve && !self->prev_s_curve;
  self->prev_s_curve = self->config.enable_s_curve;

  if (dt <= 0.0f) {
    *out_vx = target_vx;
    *out_vy = target_vy;
    *out_omega = target_omega;
    return;
  }

  if (self->seed_pending || s_curve_enabled_now) {
    TCS_Seed(self, in);
  }

  if (in->has_imu) {
    TCS_Estimate(self, in, dt);
  } else {
    self->slip_flags = 0;
    self->is_slipping = false;
    self->accel_gain = 1.0f;
  }

  if (!self->config.enable_s_curve) {
    // S字無効時は指令をそのまま通す (推定・検知はログ用に動作させておく)
    self->current_vx = target_vx;
    self->current_vy = target_vy;
    self->current_omega = target_omega;
    self->current_ax = 0.0f;
    self->current_ay = 0.0f;
    self->current_alpha = 0.0f;
  } else {
    bool intervene = self->config.enable_tcs && in->has_imu;
    float max_accel = self->config.max_accel * (intervene ? self->accel_gain : 1.0f);
    TCS_SmoothTranslation(self, target_vx, target_vy, max_accel, dt);
    if (intervene && self->is_slipping) {
      TCS_LimitToGround(self, dt);
    }
    // ω (IMUヘディングロック) はスリップ介入の対象外
    TCS_SmoothRotation(self, target_omega, dt);
  }

  *out_vx = self->current_vx;
  *out_vy = self->current_vy;
  *out_omega = self->current_omega;
}
