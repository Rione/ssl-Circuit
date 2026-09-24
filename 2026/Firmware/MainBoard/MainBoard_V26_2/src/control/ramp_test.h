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
// 結果は ramp_result (RAM) に残し、tools/read_ramp_results.ps1 が ST-Link で読む。
// 波形は tcs_log (10ms 周期、tag=何本目か、target_vx 列=フィルタ後のトルク上限 [mV]) に残る。

#define RAMP_DIR_COUNT 10     // 向きの数 (並進8 + 旋回2)
#define RAMP_MAX_STROKES 30   // 10本 × 最大3回

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
  uint16_t reserved;
  RampPhaseResult accel;
  RampPhaseResult brake;
} RampStrokeResult;

typedef struct {
  uint32_t seq;           // 走行の通し番号 (電源投入から1, 2, ...)
  uint32_t result;        // 0: 走行中、1: 最後まで、2: 安全停止、3: 取り消し
  uint16_t stroke_count;
  uint16_t retry_mask;    // bit d: 向き d の組で3回目を測った
  uint16_t unstable_mask; // bit d: 向き d は3回測っても差が大きかった
  uint16_t abort_reason;  // 1: 範囲外、2: 向きのずれ、3: 時間切れ、4: WheelUnit の異常
  RampStrokeResult stroke[RAMP_MAX_STROKES];
} RampRunResult;

extern RampRunResult ramp_result;  // 直近の1回ぶん

// 最初からやり直せるようにする (開始の直前に呼ぶ)
void RampTest_Reset(void);
// 制御周期ごとに呼ぶ。終わったら RAMP_FINISHED / RAMP_ABORTED を返す (出力は止めてある)
RampTestStatus RampTest_Step(Robot* robot);
// 取り消されたとき (Rock5A の指令など) に呼ぶ。結果に「取り消し」と記録する
void RampTest_Cancel(void);

#endif  // __RAMP_TEST_H_
