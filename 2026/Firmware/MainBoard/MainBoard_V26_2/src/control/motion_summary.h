#ifndef __MOTION_SUMMARY_H_
#define __MOTION_SUMMARY_H_

#include <stdint.h>

#include "robot.h"

// 動作パターンのテストの区間ごとの要約 (自動チューニング段階2、HANDOFF_AUTOTUNE.md 10.2)。
// 全サンプルの記録 (tcs_log.c、約62KB) は1回ぶんしか持てないので、区間ごとの数値だけを走行ごとに残す
// (1走行 436 byte)。1kHz で計算するので、30msごとの記録より細かい。
// tools/read_motion_results.ps1 が ST-Link で読んで表にする。
#define MOTION_SEG_MAX 13
#define MOTION_RESULT_RUNS 8  // 直近の走行を何回ぶん残すか (循環)

// すべて2byteの整数 (PC 側のデコードを簡単にするため)。順番・大きさを変えたら read_motion_results.ps1 も直す
typedef struct {
  uint16_t dur_ms;            // 区間の所要時間 [ms]
  uint16_t slip_pct_x10;      // TCS の is_slipping だった割合 [0.1%] (速度モード用の判定。参考)
  uint16_t lim_pct_x10;       // トルク上限で出力を縮めた割合 [0.1%]
  int16_t vmax_mmps;          // オドメトリの最高速度 [mm/s]
  // 候補のスリップ指標 (どれが2.4/2.8/3.2Vをうまく区別できるかを段階2で見て、S_ref を決める)
  int16_t s_acc_x100;         // A: 加速中の max(0, |a_odom| − |a_imu|) の平均 [0.01 m/s²] (TCS の判定とは独立)
  int16_t gap_mean_x1000;     // B: 動いている間の max(0, |v_odom| − |v_ground|) の平均 [mm/s] (推定は TCS に依存)
  uint16_t gap_frac_x1000;    // B': 上の差が 0.3m/s を超えた割合 [0.1%]
  uint16_t acc_ms;            // 加速中と数えた時間 [ms] (s_acc の分母)
  // 正確さ
  int16_t cross_max_mm;       // 進む向きからの外れの最大 [mm] (並進の区間)
  int16_t pos_over_mm;        // 目標を通り過ぎた距離の最大 [mm] (並進の区間、通り過ぎなければ0)
  int16_t head_over_mrad;     // 向きの行き過ぎの最大 [mrad] (旋回の区間は目標を越えた量、並進は向きのずれの最大)
  int16_t end_dist_mm;        // 区間が終わったときの目標までの距離 [mm]
  int16_t end_head_mrad;      // 区間が終わったときの向きの誤差 [mrad]
  // 機体の状態
  uint16_t status_or;         // WheelUnit の状態バイトの OR (4輪、bit1: 電源電圧範囲外、bit2: 過熱)
  uint16_t batt_min_x10;      // 電池電圧の最小 [0.1V]
  uint16_t reserved;
} MotionSegSummary;

typedef struct {
  uint32_t seq;               // 走行の通し番号 (1から。電源投入でリセット)
  uint32_t result;            // 0: 走行中、1: 最後まで、2: 安全停止、3: 取り消し
  uint16_t traction_x100;     // この走行で使った volt_tune の値 (×100)
  uint16_t max_accel_x100;
  uint16_t max_ang_accel_x100;
  uint16_t seg_count;         // 記録した区間の数
  uint32_t total_ms;          // 走行の所要時間 [ms]
  MotionSegSummary seg[MOTION_SEG_MAX];
} MotionRunResult;

extern MotionRunResult motion_results[MOTION_RESULT_RUNS];
extern volatile uint32_t motion_result_count;  // これまでに始めた走行の数 (最新は [(count-1) % RUNS])

// 走行の始め。volt_tune の今の値を記録する
void MotionSummary_BeginRun(void);
// 区間の始め。start は区間の始めの位置、target は目標の位置と向き (床の座標)
void MotionSummary_BeginSegment(int seg, float start_x, float start_y, float target_x,
                                float target_y, float target_heading, float start_heading);
// 制御周期ごとに呼ぶ。pos_x, pos_y, heading は床の座標での位置 [m] と向き [rad]
void MotionSummary_Step(float dt, const Robot* robot, float pos_x, float pos_y, float heading);
// 区間の終わり (次の区間へ進む、安全停止のとき)。pos と heading は終わったときの値
void MotionSummary_EndSegment(float pos_x, float pos_y, float heading, uint32_t dur_ms);
// 走行の終わり。result: 1: 最後まで、2: 安全停止、3: 取り消し
void MotionSummary_EndRun(uint32_t result, uint32_t total_ms);

#endif  // __MOTION_SUMMARY_H_
