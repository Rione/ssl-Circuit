#ifndef __VOLT_TUNE_H_
#define __VOLT_TUNE_H_

#include <stdint.h>

// 電圧制御の調整できる値 (実行中に書き換えられる)。既定値は parammeter.h の同名の #define。
// 自動チューニング (auto_tune.c) が走行ごとに書き換え、合格した値はフラッシュに保存して試合でも使う
// (引き継ぎ文書 HANDOFF_AUTOTUNE.md)。ST-Link から書き換えてもよいが、使う前に VoltTune_Sanitize で
// 安全範囲に収める
#define VOLT_TUNE_MAGIC 0x56545031U  // "VTP1"
#define VOLT_TUNE_VERSION 1U

typedef struct {
  uint32_t magic;
  uint32_t version;
  float traction_limit_v;  // トルク上限 [V] (VOLT_TRACTION_LIMIT_V)
  float max_accel;         // S字の加速度上限 [m/s^2] (VOLT_MODE_MAX_ACCEL)
  float max_ang_accel;     // S字の角加速度上限 [rad/s^2] (VOLT_MODE_MAX_ANG_ACCEL)
  float max_jerk;          // S字のジャーク上限 [m/s^3] (TCS_MAX_JERK)
  float max_ang_jerk;      // S字の角ジャーク上限 [rad/s^3] (TCS_MAX_ANG_JERK)
  float ka_lin;            // 加減速ぶんのFF 並進 [V/(m/s^2)] (WHEEL_VOLT_KA_LIN_BODY)
  float ka_ang;            // 加減速ぶんのFF 回転 [V/(rad/s^2)] (WHEEL_VOLT_KA_ANG_BODY)
  float kp_lin;            // 機体速度PI 並進 P [V/(m/s)] (VEL_FB_KP_LIN)
  float ki_lin;            // 並進 I [V/(m/s·s)] (VEL_FB_KI_LIN)
  float kp_ang;            // 回転 P [V/(rad/s)] (VEL_FB_KP_ANG)
  float ki_ang;            // 回転 I [V/(rad/s·s)] (VEL_FB_KI_ANG)
  float i_max_v;           // 積分項の上限 [V] (VEL_FB_I_MAX_V)
} VoltTuneParams;

// 今使っている値 (電圧制御の分岐と OmniDrive_SetControlMode が毎周期読む)
extern VoltTuneParams volt_tune;

// 既定値 (parammeter.h) に戻す
void VoltTune_SetDefaults(VoltTuneParams* p);
// 各値を安全範囲に収める。magic・版が合わない、または NaN を含むときは既定値に戻す。
// 戻り値: 何か直したら 1
int VoltTune_Sanitize(VoltTuneParams* p);

#endif  // __VOLT_TUNE_H_
