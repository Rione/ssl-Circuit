#ifndef __RAMP_TEST_H_
#define __RAMP_TEST_H_

#include <stdint.h>

#include "robot.h"

// ランプ試験 (自動チューニング、HANDOFF_AUTOTUNE.md 10.7): 止まった状態から、トルク上限 (volt_tune の
// traction_limit_v) を約0.28秒で 1.8V→3.2V までなめらかに上げて加速し、車輪と IMU の加速度が離れ始めた
// ときのトルク (滑り始め、主な指標) と、IMU の加速度が一番大きくなったときのトルク (ピーク) を向きごとに測る。
// 続けてブレーキも同じように測る。
//  - 向き: 前・後・左・右・斜め4方向・その場旋回の左右 (10本)。2本ずつ組にして、1本目は原点から
//    空きのある方へ、2本目は戻る向きへ走る。組ごとに低速で原点に戻る
//  - これを2回くり返し、滑り始めの差が大きい向きの組だけ3回目を測る。3回目でも差が大きい向きは
//    「不安定」と記録する
//  - トルクは止まった状態から短時間で上げる。速度が出ると、今の回転数で転がり続ける電圧が増えて
//    電圧上限 (4.9V) までの余裕が減り、トルクを上げられなくなるため
// 速度別の測定 (speed_mask): 各速度 v0 まで巡航してからランプ加速し、続けてブレーキ (停止距離を測る)。
// (ブレーキのみの本は、結果の accel の終わり方が RAMP_END_NONE、v0_x100 > 0 で見分ける)
// 各本の前に、必要な長さを予測し、範囲 (x: −1.0〜3.5m、y: ±2.5m の端から 0.3m 内側) に収まる位置から走る。
// 収まらない向きは走らせずに「範囲不足」と記録する。
// 結果は ramp_result (RAM) に残し、tools/read_ramp_results.ps1 が ST-Link で読む。
// 走行の終わりには、原点 (範囲の中心)・向き 0 へ戻ってから終わる (次の走行が、位置のずれを引きずらないように)。
// 波形は tcs_log (10ms 周期、tag=何本目か、target_vx 列=フィルタ後のトルク上限 [mV]) に残る。

#define RAMP_DIR_COUNT 10     // 向きの数 (並進8 + 旋回2)
// 記録できる本数: 低速 (v0=0) の 10本 × 最大3回 = 30 + 速度別の最大 20本 (+ 余裕)
#define RAMP_MAX_STROKES 56

// 速度別の測定 (速度の指定 speed_mask、HANDOFF_AUTOTUNE.md 10.10)。bit0 (1) は従来の低速 (止まった状態から)。
// 0 を渡すと 1 (従来どおり) になる。速度の段は、その速度まで巡航してからランプを掛ける
#define RAMP_SPEED_V0       0x01U  // v0=0: 従来の10向き (止まった状態から)
#define RAMP_SPEED_AXIS_1_0 0x02U  // 前後左右 1.0 m/s
#define RAMP_SPEED_DIAG_1_5 0x04U  // 斜め 1.5 m/s
#define RAMP_SPEED_AXIS_2_0 0x08U  // 前後左右 2.0 m/s
#define RAMP_SPEED_AXIS_3_0 0x10U  // 前後左右 3.0 m/s (前後は、範囲に収まると予測したときだけ)
#define RAMP_SPEED_DIAG_2_0 0x20U  // 斜め 2.0 m/s
// ブレーキのみ (加速のランプ無し): v0 まで巡航してから、直接ブレーキ。停止距離と減速度のピークを測る。
// ブレーキは 1.0V から上げ、IMU の減速度がピークを過ぎて落ち始めたら、そのトルクで保つ (ロック判定は当てにならない)
#define RAMP_SPEED_BRAKE_2_0 0x40U  // 前後 2.0 m/s (ブレーキのみ)
#define RAMP_SPEED_BRAKE_2_5 0x100U // 前後 2.5 m/s (ブレーキのみ。長辺 3.5m のエリアでは、予測が範囲不足になりやすい)
#define RAMP_SPEED_BRAKE_2_0L 0x200U // 左右 2.0 m/s (ブレーキのみ。左右は助走が長く、3.5×2.5m のエリアでは走らせられない)
// FF 試験 (HANDOFF_AUTOTUNE.md 10.11): PI を切って FF だけで、指令の加速度 (1.5 と 2.5 m/s²) を出し、実際の
// 加速度 (IMU) との比を測る。向き (前・後・左・右) ごと。比が 1 より小さければ、その向きの FF (ka) が足りない。
// 目標速度 1.5m/s、1本は約 1.0〜1.7m。結果は ff_results (FfStepResult)。tools/read_ff_results.ps1 で読む
#define RAMP_SPEED_FF        0x400U
// FF 試験の軽量版 (自動最適化 optimizer.c が使う): 指令の加速度 2.5m/s² だけ、前・後・左・右 × 2回 = 8本 (約50秒)。
// 3.5×2.5m のエリアの1回目の向きだけで、4方向とも収まる (回転しない)
#define RAMP_SPEED_FF_FAST   0x1000U
// 速度別の測定を1回終えたら、機体が自分で範囲の真ん中へ移動し、右へ 90° 回って、もう一度繰り返す (2回目)。
// 2回目の範囲は、長方形の縦と横を入れ替えて、1回目から自動で求める (1回目の範囲は、長方形から各端 0.2m 内側で、
// 原点は範囲の x の中心、y は左右の真ん中、を前提にする)。低速 (v0=0) は繰り返さない。
// 結果の set 欄は、速度別では 1回目=1、2回目=2
#define RAMP_SPEED_ROTATE   0x80U

typedef enum {
  RAMP_DIR_FWD = 0,        // +x
  RAMP_DIR_BACK = 1,       // −x
  RAMP_DIR_LEFT = 2,       // +y
  RAMP_DIR_RIGHT = 3,      // −y
  RAMP_DIR_FWD_LEFT = 4,   // +x+y
  RAMP_DIR_BACK_RIGHT = 5, // −x−y
  RAMP_DIR_FWD_RIGHT = 6,  // +x−y
  RAMP_DIR_BACK_LEFT = 7,  // −x+y
  RAMP_DIR_ROT_CCW = 8,    // 左回り (+ω)
  RAMP_DIR_ROT_CW = 9,     // 右回り (−ω)
} RampDir;

// 加速・ブレーキの終わり方
typedef enum {
  RAMP_END_NONE = 0,
  RAMP_END_ONSET = 1,     // 滑り始めを見つけた (加速は、その後ピークを測ってからやめた)
  RAMP_END_MAX_V = 2,     // 上限 (3.2V) に達した
  RAMP_END_SPEED = 3,     // 速度の上限に達した
  RAMP_END_HEADROOM = 4,  // 電圧上限までの余裕がトルク上限より小さくなった (これ以上は測れない)
  RAMP_END_STOPPED = 5,   // ブレーキ: 滑る前に止まった
  RAMP_END_TIMEOUT = 6,
  RAMP_END_RANGE = 7,     // 必要な長さが使える範囲に収まらないと予測したので、走らせなかった
} RampEndReason;

typedef enum {
  RAMP_RUNNING = 0,
  RAMP_FINISHED = 1,
  RAMP_ABORTED = 2,
} RampTestStatus;

// 1回の加速かブレーキの結果 (すべて2byte、14byte)。加速度の単位は、並進は [0.01 m/s²]、旋回は [0.1 rad/s²]
typedef struct {
  uint16_t onset_v_x100;  // 滑り始めのトルク上限 [0.01V] (0: 見つからなかった)
  uint16_t peak_v_x100;   // IMU の加速度が一番大きかったときのトルク上限 [0.01V]
  int16_t peak_acc;       // そのときの IMU の加速度 (大きさ)
  int16_t onset_acc;      // 滑り始めのときの IMU の加速度 (大きさ)
  int16_t speed;          // 終わったときの速度 [mm/s] (旋回は [mrad/s])
  uint16_t end_reason;    // RampEndReason
  int16_t avg_acc;        // ピークを更新したときの、直近 100ms の IMU の加速度の平均 (継続する加速度の目安)
} RampPhaseResult;

typedef struct {
  uint8_t dir;            // RampDir
  uint8_t set;            // 何回目のくり返しか (1〜3)
  uint8_t status_or;      // WheelUnit の状態バイトの OR (4輪)
  uint8_t batt;           // 電池電圧の最小 [V]
  uint16_t t_start_ms;    // 試験の開始からの時刻 [ms] (tcs_log と照らし合わせる)
  uint16_t v0_x100;       // ランプを掛ける前の巡航の速度 [0.01 m/s] (0: 止まった状態から)
  RampPhaseResult accel;
  RampPhaseResult brake;
  // 速度別の測定 (並進のみ)。距離は、その相の間に進行方向へ進んだ距離 [mm]
  uint16_t v_brake_start;  // ブレーキを始めたときの速度 [mm/s] (ランプで加速したあと)
  uint16_t brake_dist_mm;  // ブレーキで止まるまでの距離 (停止距離表)。走らせなかったとき 0
  uint16_t accel_dist_mm;  // ランプ加速の間の距離
  uint16_t cruise_dist_mm; // 助走 (v0 まで巡航) の距離。走らせなかったときは、予測した必要な長さ [mm]
} RampStrokeResult;

typedef struct {
  uint32_t seq;           // 走行の通し番号 (電源投入から1, 2, ...)
  uint32_t result;        // 0: 走行中、1: 最後まで、2: 安全停止、3: 取り消し
  uint16_t stroke_count;
  uint16_t retry_mask;    // bit d: 向き d の組で3回目を測った
  uint16_t unstable_mask; // bit d: 向き d は3回測っても差が大きかった
  uint16_t abort_reason;  // 1: 範囲外、2: 向きのずれ、3: 時間切れ、4: WheelUnit の異常
  uint32_t speed_mask;    // この走行で指定した速度の段 (RAMP_SPEED_*)
  RampStrokeResult stroke[RAMP_MAX_STROKES];
} RampRunResult;

extern RampRunResult ramp_result;  // 直近の1回ぶん

// FF 試験の1本の結果 (16 byte)。加速度の単位はすべて [0.01 m/s²]
#define FF_RESULT_MAX 48
typedef struct {
  uint8_t dir;            // 向き (RampDir。0: 前、1: 後、2: 左、3: 右。機体から見た向き)
  uint8_t session;        // 1: 1回目、2: 機体を 90° 回した2回目
  uint16_t a_nom_x100;    // 指令した加速度の上限 (S字の max_accel) [0.01 m/s²]
  int16_t a_cmd_x100;     // 窓の間の、S字が実際に出した加速度の平均 (進む向き)
  int16_t a_imu_x100;     // 窓の間の、IMU の加速度の平均 (進む向き)。実際に出た加速度
  int16_t a_odom_x100;    // 窓の間の、車輪 (オドメトリ) の加速度の平均。a_imu より大きければ空転
  uint16_t win_ms;        // 平均を取った窓の長さ [ms] (0: 取れなかった = 範囲不足・届かなかった)
  int16_t v_end_mmps;     // 窓の終わりの速度 (オドメトリ) [mm/s]
  uint8_t status_or;      // WheelUnit の状態バイトの OR
  uint8_t valid;          // 1: 有効、0: 走らなかった (範囲不足) / 窓が短い
} FfStepResult;

// WheelUnit の異常 (状態バイトの bit1: 電源電圧範囲外、bit2: 過熱) で止めたときの、そのときの状態 (4輪の状態バイト、
// 輪 i が (ramp_abort_status >> (8*i)) & 0xFF) と電池電圧 [V]。安全停止の原因を調べるための記録
extern volatile uint32_t ramp_abort_status;
extern volatile uint16_t ramp_abort_batt;

extern FfStepResult ff_results[FF_RESULT_MAX];
extern volatile uint16_t ff_result_count;

// 速度別の測定の範囲 [m] (原点はスタート位置): x_min (負)〜x_max、y は ±y_abs。常識的な範囲に収まらない値は既定
// (x: −1.0〜3.5、y: ±2.5) に戻す。各本の経路は、この範囲の端から 0.3m 内側に収まる位置から走る
void RampTest_SetArea(float x_min, float x_max, float y_abs);
// 最初からやり直せるようにする (開始の直前に呼ぶ)。speed_mask: RAMP_SPEED_* の組み合わせ (0 は RAMP_SPEED_V0)
void RampTest_Reset(uint32_t speed_mask);
// 制御周期ごとに呼ぶ。終わったら RAMP_FINISHED / RAMP_ABORTED を返す (出力は止めてある)
RampTestStatus RampTest_Step(Robot* robot);
// 取り消されたとき (Rock5A の指令など) に呼ぶ。結果に「取り消し」と記録する
void RampTest_Cancel(void);

#endif  // __RAMP_TEST_H_
