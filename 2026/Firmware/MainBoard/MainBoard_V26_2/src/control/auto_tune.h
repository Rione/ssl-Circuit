#ifndef __AUTO_TUNE_H_
#define __AUTO_TUNE_H_

#include <stdbool.h>
#include <stdint.h>

#include "local_controller.h"
#include "robot.h"

// 自動チューニング (引き継ぎ文書 HANDOFF_AUTOTUNE.md)。
// 今は「ST-Link から開始を指示したときだけテストを1回走らせる」入口だけを持つ (段階1)。
// 開始の手順 (tools/autotune_start.ps1):
//   PC が test_id → magic → start_seq (= done_seq + 1) の順に RAM へ書く → ケーブルを抜いて離れる
//   → 10秒後 (LED0 が速く点滅) に走り出す → 終わると done_seq = start_seq、result に結果を入れて止まる
// autotune_ctrl は bss にあり、起動時は0なので勝手には始まらない。
// Rock5A から指令・緊急停止を受けたら AutoTune_Cancel で待ち・走行を取り消す
#define AUTOTUNE_CTRL_MAGIC 0x41545331U  // "ATS1"

typedef enum {
  AUTOTUNE_TEST_NONE = 0,
  AUTOTUNE_TEST_MOTION_PATTERN = 1,  // 動作パターンのテストを1周 (今の値のまま)
  AUTOTUNE_TEST_RAMP = 2,            // ランプ試験 (向きごとの滑り始め・限界を測る、ramp_test.h)
  AUTOTUNE_TEST_OPTIMIZE = 4,        // 自動最適化 (機体が試験をくり返して、値を更新・検証・保存する、optimizer.h)
  AUTOTUNE_TEST_CLEAR_SAVED = 5,     // フラッシュに保存された調整値を消す (IMU の較正値は残す)。待たずに実行
  AUTOTUNE_TEST_MOTION_BATCH = 7,    // 動作パターンを、ka_lat を変えながら続けて走らせる (0.75, 0.5, 0.75, 0.5。比べるための試験)
  AUTOTUNE_TEST_BEEP = 6,            // ブザーの確認 (成功の音を鳴らす。走らない)
} AutoTuneTestId;

typedef enum {
  AUTOTUNE_STATE_IDLE = 0,
  AUTOTUNE_STATE_WAITING = 1,  // 開始を受け、走り出すまで待っている
  AUTOTUNE_STATE_RUNNING = 2,
} AutoTuneState;

typedef enum {
  AUTOTUNE_RESULT_NONE = 0,
  AUTOTUNE_RESULT_FINISHED = 1,   // 最後まで走った
  AUTOTUNE_RESULT_ABORTED = 2,    // テストの安全停止
  AUTOTUNE_RESULT_CANCELLED = 3,  // Rock5A の指令・緊急停止で取り消した
  AUTOTUNE_RESULT_BAD_TEST = 4,   // test_id が不正
} AutoTuneResult;

// PC (ST-Link) と共有する領域。すべて32bit (STM32_Programmer_CLI の -r32 / -w32 で読み書きする)
typedef struct {
  uint32_t magic;      // PC が AUTOTUNE_CTRL_MAGIC を書く
  uint32_t start_seq;  // PC が done_seq + 1 を書くと1回走る (最後に書く)
  uint32_t done_seq;   // MainBoard が、終わった (取り消した) ときに start_seq を写す
  uint32_t test_id;    // AutoTuneTestId
  uint32_t state;      // AutoTuneState (MainBoard が書く)
  uint32_t result;     // AutoTuneResult (MainBoard が書く。直近の1回ぶん)
  // 開始の指示と一緒に PC が書く、この1回だけの volt_tune の上書き (0: 上書きしない = 既定値)。
  // 走り終わる (取り消す) と既定値に戻る。範囲は VoltTune_Sanitize が収める。段階2で 2.4/2.8/3.2V を試すのに使う
  uint32_t traction_x100;       // トルク上限 [0.01V]
  uint32_t max_accel_x100;      // S字の加速度上限 [0.01 m/s^2]
  uint32_t max_ang_accel_x100;  // S字の角加速度上限 [0.01 rad/s^2]
  // ランプ試験 (test_id=2) の速度の段 (RAMP_SPEED_* の組み合わせ。0 は低速だけ = 従来どおり)
  uint32_t ramp_speed_mask;
  // ランプ試験の速度別の測定の範囲 [cm] (原点はスタート位置)。ramp_x_max_cm = 0 なら既定 (前 3.5m・後ろ 1.0m・左右 2.5m)
  int32_t ramp_x_min_cm;  // 後ろ (負)
  int32_t ramp_x_max_cm;  // 前
  int32_t ramp_y_abs_cm;  // 左右 (片側)
  // 左右の FF 係数 ka_lat の上書き [0.001 V/(m/s^2)] (0: 既定値 = WHEEL_VOLT_KA_LAT_BODY)。FF の較正を、再書き込みなしで試すのに使う
  uint32_t ka_lat_x1000;
  // 自動最適化 (test_id=4): 最適化するタスク (OPT_TASK_*、0 は FF 係数) と、OPT_FLAG_* (保存の許可・比の基準)
  uint32_t opt_task_mask;
  uint32_t opt_flags;
} AutoTuneCtrl;

extern volatile AutoTuneCtrl autotune_ctrl;

// Rock5A 未接続の分岐から毎周期呼ぶ。開始の指示を受けて待っている間・走っている間は出力を持って
// true を返す。それ以外は何もせず false を返す (呼び出し側で LocalController_Stop する)
bool AutoTune_Poll(LocalController* lc, Robot* robot);
// Rock5A の指令・緊急停止を受けたときに呼ぶ。待ち・走行中、および未処理の開始の指示を取り消す
void AutoTune_Cancel(void);

#endif  // __AUTO_TUNE_H_
