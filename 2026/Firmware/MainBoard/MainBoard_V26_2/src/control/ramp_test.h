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
// 各本の前に、必要な長さを予測し、範囲 (x: −1.0〜3.5m、y: ±2.5m の端から 0.3m 内側) に収まる位置から走る。
// 収まらない向きは走らせずに「範囲不足」と記録する。
// 結果は ramp_result (RAM) に残し、tools/read_ramp_results.ps1 が ST-Link で読む。
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

// 1回の加速かブレーキの結果 (すべて2byte)。加速度の単位は、並進は [0.01 m/s²]、旋回は [0.1 rad/s²]
typedef struct {
  uint16_t onset_v_x100;  // 滑り始めのトルク上限 [0.01V] (0: 見つからなかった)
  uint16_t peak_v_x100;   // IMU の加速度が一番大きかったときのトルク上限 [0.01V]
  int16_t peak_acc;       // そのときの IMU の加速度 (大きさ)
  int16_t onset_acc;      // 滑り始めのときの IMU の加速度 (大きさ)
  int16_t speed;          // 終わったときの速度 [mm/s] (旋回は [mrad/s])
  uint16_t end_reason;    // RampEndReason
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
