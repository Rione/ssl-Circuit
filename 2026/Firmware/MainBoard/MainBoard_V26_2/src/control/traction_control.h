#ifndef __TRACTION_CONTROL_H_
#define __TRACTION_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "parammeter.h"

#ifdef __cplusplus
extern "C" {
#endif

// スリップ要因ビット (TractionControl.slip_flags)
#define TCS_SLIP_GEOM 0x01U   // 4輪幾何拘束残差
#define TCS_SLIP_ROT 0x02U    // 旋回残差 (オドメトリ - ジャイロ)
#define TCS_SLIP_ACCEL 0x04U  // 加速度残差 (オドメトリ - IMU)

// TCS 設定パラメータ構造体
typedef struct {
  float max_accel;        // 最大並進加速度 [m/s^2]
  float max_jerk;         // 最大並進ジャーク [m/s^3]
  float max_ang_accel;    // 最大角加速度 [rad/s^2]
  float max_ang_jerk;     // 最大角ジャーク [rad/s^3]

  float geom_k;             // 幾何拘束係数: sqrt(2)*sin(55deg)
  float geom_slip_thresh;   // 幾何学的拘束残差スリップ判定閾値 [rad/s]
  float rot_slip_thresh;    // 旋回ジャイロ残差スリップ判定閾値 [rad/s]
  float accel_slip_thresh;  // 加速度残差スリップ判定閾値 [m/s^2]
  float accel_detect_min;   // 加速度残差で判定する最小指令加速度 [m/s^2]
  float vel_slip_exit;      // スリップ解除: |v_odom - v_ground| の上限 [m/s]
  float slip_hold_s;        // スリップ解除: 要因が消えてからの保持時間 [s]
  float slip_vel_margin;    // スリップ中に許す 指令速度 - 対地速度 [m/s]

  float ground_vel_kc;     // 対地速度推定の相補フィルタゲイン [1/s]
  float accel_lpf_hz;      // 加速度LPFカットオフ [Hz]
  float max_inertial_s;    // IMU積分のみで対地速度を推定する最大時間 [s]

  float min_gain;         // スリップ時の加速度上限比率の下限
  float slip_gain_cut;    // スリップ突入1回あたりの加速度上限比率の最大カット率
  float recovery_rate;    // グリップ回復後の加速度上限比率の復帰速度 [1/s]
  float deadzone_speed;   // 低速デッドゾーン [m/s]

  bool enable_tcs;        // スリップ介入 有効/無効 (無効でも推定・検知は動作しログに残る)
  bool enable_s_curve;    // S字加減速制限 有効/無効
} TCSConfig;

// 1制御周期ぶんのセンサ入力
typedef struct {
  float wheel_vel[4];     // 実測車輪角速度 [rad/s]
  float odom_vx;          // 順運動学による機体速度X [m/s]
  float odom_vy;          // 順運動学による機体速度Y [m/s]
  float odom_omega;       // 順運動学による機体角速度 [rad/s]
  float gyro_yaw_rate;    // IMU角速度 [rad/s]
  float accel_x;          // IMU加速度 機体座標X [m/s^2]
  float accel_y;          // IMU加速度 機体座標Y [m/s^2]
  bool has_imu;           // false の場合はスリップ検知・介入を行わない (S字のみ)
} TCSInput;

// TCS 状態管理構造体
typedef struct {
  TCSConfig config;

  // S字加減速・平滑化用内部状態
  float current_vx;       // 平滑化後の並進速度X [m/s]
  float current_vy;       // 平滑化後の並進速度Y [m/s]
  float current_omega;    // 平滑化後の回転角速度 [rad/s]

  float current_ax;       // 現在の並進加速度X [m/s^2]
  float current_ay;       // 現在の並進加速度Y [m/s^2]
  float current_alpha;    // 現在の角加速度 [rad/s^2]

  bool seed_pending;      // 次回更新時に平滑化状態を実測速度で初期化する
  bool prev_s_curve;      // 前回周期の enable_s_curve (無効→有効の切替検出用)

  // 対地速度推定 (IMU加速度積分 + グリップ中はオドメトリへ相補補正)
  float odom_vx, odom_vy;      // 直近のオドメトリ速度 [m/s]
  float ground_vx, ground_vy;  // 推定対地速度 [m/s]
  float a_odom_x, a_odom_y;    // LPF済みオドメトリ加速度 [m/s^2]
  float a_imu_x, a_imu_y;      // LPF済みIMU加速度 [m/s^2]

  // スリップ状態
  float geom_residual;    // 4輪幾何学的拘束残差 [rad/s]
  float rot_residual;     // 旋回残差 (オドメトリ - ジャイロ) [rad/s]
  float accel_residual;   // 加速度残差 (a_odom - a_imu の指令加速度方向成分) [m/s^2]
  uint8_t slip_flags;     // 今周期に閾値を超えた要因 (TCS_SLIP_*)
  bool is_slipping;       // スリップ状態 (解除はヒステリシス付き)
  bool prev_slipping;     // 前周期の is_slipping (スリップ突入の検出用)
  float slip_time_s;      // スリップ継続時間 [s]
  float clear_time_s;     // スリップ中に要因が消えてからの経過時間 [s]

  // 能動介入: S字の加速度上限に掛ける比率 (1.0=通常, <1.0=抑制中)
  float accel_gain;
} TractionControl;

// 初期化
void TCS_Init(TractionControl* self);

// 状態のリセット (停止時など)。平滑化状態は次回 TCS_Update 時に実測速度から初期化される
void TCS_Reset(TractionControl* self);

// 1制御周期の処理: 対地速度推定 → スリップ検知 → スリップ量に応じたS字加減速
// target_* は上位からの目標速度、out_* は逆運動学に渡す速度指令
void TCS_Update(TractionControl* self, const TCSInput* in, float target_vx,
                float target_vy, float target_omega, float* out_vx,
                float* out_vy, float* out_omega, float dt);

#ifdef __cplusplus
}
#endif

#endif  // __TRACTION_CONTROL_H_
