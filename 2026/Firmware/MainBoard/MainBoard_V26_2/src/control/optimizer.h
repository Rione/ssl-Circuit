#ifndef __OPTIMIZER_H_
#define __OPTIMIZER_H_

#include <stdbool.h>
#include <stdint.h>

#include "local_controller.h"
#include "robot.h"

// 自動最適化 (HANDOFF_AUTOTUNE.md 10.12): 一度 ST-Link から開始を指示すると、機体が自分で試験をくり返して
// 値を更新し、収束したら検証し、合格したらフラッシュに保存する (auto_tune.c の test_id=4 から呼ばれる)。
// 今のタスクは「FF 係数 (ka_lin、ka_lat)」だけ: FF 試験の軽量版 (RAMP_SPEED_FF_FAST) で、指令の加速度に対する
// 実際の加速度の比 r を、前後と左右で測り、r = 1 になるように ka を直す。
//   r ∝ ka^e (e ≈ 0.7。負荷分の電圧が一定のため)。1回目は e=0.7 として、以降は直前の2点から e を推定する。
// 収束: 前後・左右とも |r−1| ≤ 0.03 → 同じ値でもう1回測り (検証)、|r−1| ≤ 0.05 なら合格。
// 走行中の異常 (安全停止) は、位置が分からなくなるので、やり直さずに失敗として止まる (値は変えない)。

// opt_flags (autotune_ctrl.opt_flags)
#define OPT_FLAG_SAVE 0x01U     // 合格したらフラッシュに保存する (無ければ、値は結果に残すだけ)
#define OPT_FLAG_REF_IMU 0x02U  // 比の基準を IMU にする (既定は車輪。IMU は車輪の約 0.77 倍を示すので、ka が大きくなる)

// opt_task_mask
#define OPT_TASK_FF 0x01U

typedef enum {
  OPT_RESULT_RUNNING = 0,
  OPT_RESULT_SAVED = 1,          // 合格して保存した
  OPT_RESULT_PASSED_NOSAVE = 2,  // 合格 (保存は許可されていない。値は結果に残る)
  OPT_RESULT_NO_CONVERGE = 3,    // 反復の上限までに収束しなかった (値は変えない)
  OPT_RESULT_ABORTED = 4,        // 走行の安全停止
  OPT_RESULT_BAD_DATA = 5,       // 測れなかった・比が範囲外 (センサの異常の疑い)
  OPT_RESULT_PRECHECK = 6,       // 電池電圧・WheelUnit の異常・過熱が解消しない
  OPT_RESULT_TIMEOUT = 7,        // 全体の時間の上限
  OPT_RESULT_CANCELLED = 8,
  OPT_RESULT_SAVE_FAILED = 9,    // 合格したが、フラッシュへの保存に失敗した
} OptResultCode;

#define OPT_LOG_MAX 16
// 1回の走行の記録 (24 byte)
typedef struct {
  uint16_t kind;               // 0: 反復、1: 検証
  uint16_t ramp_result;        // 走行の結果 (1: 最後まで、2: 安全停止)
  int16_t r_fb_x1000;          // 前後の比 [0.001] (0: 測れなかった)
  int16_t r_lat_x1000;         // 左右の比
  uint16_t ka_lin_x1000;       // この走行で使った値 [0.001 V/(m/s²)]
  uint16_t ka_lat_x1000;
  uint16_t valid_count;        // 有効だった本数 (8 本中)
  uint16_t reserved;
  uint32_t t_ms;               // 開始からの時刻 [ms]
  uint16_t next_ka_lin_x1000;  // この結果で更新した値
  uint16_t next_ka_lat_x1000;
} OptIterLog;

// PC (ST-Link) が読む結果 (tools/read_opt_results.ps1)
typedef struct {
  uint32_t seq;     // 最適化の通し番号 (電源投入から 1, 2, ...)
  uint32_t state;   // 0: 走っていない、1: 走行中
  uint32_t result;  // OptResultCode
  uint32_t elapsed_ms;
  uint16_t run_count;  // 走らせた回数 (反復 + 検証)
  uint16_t verify_count;
  uint16_t saved;  // 1: フラッシュに保存した
  uint16_t flags;  // 開始時の opt_flags
  uint16_t start_ka_lin_x1000;
  uint16_t start_ka_lat_x1000;
  uint16_t final_ka_lin_x1000;
  uint16_t final_ka_lat_x1000;
  OptIterLog log[OPT_LOG_MAX];
} OptResult;

extern volatile OptResult opt_result;

typedef enum {
  OPT_RUNNING = 0,
  OPT_DONE = 1,  // opt_result.result に結果
} OptStatus;

// 開始の直前に呼ぶ (待ちが終わったとき)。今の volt_tune を開始の値にする
void Optimizer_Begin(uint32_t task_mask, uint32_t flags);
// 制御周期ごとに呼ぶ。終わったら OPT_DONE (出力は止めてある)
OptStatus Optimizer_Step(LocalController* lc, Robot* robot);
// 取り消されたとき (Rock5A の指令など) に呼ぶ
void Optimizer_Cancel(void);

#endif  // __OPTIMIZER_H_
