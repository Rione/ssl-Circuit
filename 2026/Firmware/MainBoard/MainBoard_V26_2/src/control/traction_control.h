#ifndef __TRACTION_CONTROL_H_
#define __TRACTION_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "parammeter.h"

#ifdef __cplusplus
extern "C" {
#endif

// TCS 設定パラメータ構造体
typedef struct {
  float max_accel;        // 最大並進加速度 [m/s^2]
  float max_jerk;         // 最大並進ジャーク [m/s^3]
  float max_ang_accel;    // 最大角加速度 [rad/s^2]
  float max_ang_jerk;     // 最大角ジャーク [rad/s^3]

  float geom_k;           // 幾何拘束係数: sqrt(2)*sin(55deg)
  float geom_slip_thresh; // 幾何学的拘束残差スリップ判定閾値 [rad/s]
  float rot_slip_thresh;  // 旋回ジャイロ残差スリップ判定閾値 [rad/s]

  float slip_gain;        // スリップ検知時の介入抑制ゲイン
  float min_gain;         // 最小抑制ゲイン (出力の下限比率)
  float recovery_rate;    // グリップ回復後のゲイン復帰速度 [1/s]
  float nominal_voltage;  // 基準バッテリー電圧 [V]

  bool enable_tcs;        // TCS有効/無効フラグ
  bool enable_s_curve;    // S字加減速制限有効/無効フラグ
} TCSConfig;

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

  // スリップ状態
  float geom_residual;    // 4輪幾何学的拘束残差 [rad/s]
  float rot_residual;     // 旋回残差 (オドメトリ - ジャイロ) [rad/s]
  float wheel_slip[4];    // 各輪のスリップ推定残差 [rad/s]
  bool is_slipping[4];    // 各輪のスリップ検知フラグ
  bool is_any_slipping;   // いずれかの輪がスリップしているか

  // 能動介入ゲイン (1.0: 通常, < 1.0: 出力抑制中)
  float wheel_gain[4];    // 各輪の出力スケーリングゲイン
} TractionControl;

// 初期化
void TCS_Init(TractionControl* self);

// 状態のリセット (停止時など)
void TCS_Reset(TractionControl* self);

// 速度指令の平滑化 (ジャーク・加速度制限付きS字フィルタ)
void TCS_SmoothVelocity(TractionControl* self, float target_vx, float target_vy,
                        float target_omega, float* out_vx, float* out_vy,
                        float* out_omega, float dt);

// スリップ検知 (4輪幾何拘束 & IMUジャイロ照合)
void TCS_DetectSlip(TractionControl* self, const float actual_wheel_vel[4],
                    float gyro_yaw_rate, float dt);

// 能動トラクション制御の適用 (スリップに応じた出力抑制と滑らかな復帰)
void TCS_ApplyIntervention(TractionControl* self, float target_wheel_vel[4],
                           float dt);

// 電圧補正 (バッテリー電圧に応じたスケーリング)
void TCS_CompensateVoltage(const TractionControl* self, float wheel_vel[4],
                           float battery_voltage);

#ifdef __cplusplus
}
#endif

#endif  // __TRACTION_CONTROL_H_
