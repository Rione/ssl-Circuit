#include "ramp_test.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "mymath.h"
#include "tcs_log.h"
#include "timer.h"
#include "volt_tune.h"
#include "wheel_voltage.h"

RampRunResult ramp_result;
FfStepResult ff_results[FF_RESULT_MAX];
volatile uint16_t ff_result_count = 0;
volatile uint32_t ramp_abort_status = 0;
volatile uint16_t ramp_abort_batt = 0;
// WheelUnit の状態バイトの異常 (bit1: 電源電圧範囲外、bit2: 過熱) が、この周期数 (1ms) 続いたら安全停止する。
// 1フレームだけの異常 (受信の同期ずれで、状態バイトが化ける) で止まらないように (FF 試験の2回目が、原因が分からない
// まま、始点への移動の最初に「WheelUnit の異常」で止まった)
#define RAMP_STATUS_DEBOUNCE_CYCLES 20

// ---- 試験の定数 ----
// トルク上限のランプ (2026-09-24 の初回の結果で見直し、HANDOFF_AUTOTUNE.md 10.8・10.9)。
//  - 始めは 1.8V: どの向きも 2.1V より下では滑らなかったので、その手前は急いで通り過ぎる
//  - 上限は 3.2V (volt_tune の安全範囲の上限と同じ): 採用する値は前後左右 (2.1〜2.5V) で決まる
//  - 速さは 5V/s (1.8→3.2V を約0.28秒): 初回の 10V/s の半分。ゆっくりすぎると速度が出て、今の回転数で
//    転がり続ける電圧が増え、電圧上限 (4.9V) までの余裕が足りなくなる
#define RAMP_START_V 1.8f
// (2026-09-24 ユーザー決定: 斜めは限界まで上げてよい) 上限は 4.5V。物理・安全面: 出力の電圧は omni_drive.c で
// min(4.9V − 回転数分の電圧, トルク上限) に収まるので、この値を上げても WheelUnit へ 4.9V は超えない
// (WheelUnit の保護は +5.0V が1秒続いたとき。ランプは1本1秒未満)。過熱・電源異常は状態バイトで見て、立てば止める。
// 滑り始めを見つけたら加速は約0.4V上げて止まる。採用する値の安全範囲 (volt_tune.c の 3.2V) は変えない。
// 斜めが 3.2V に届かなかった本は、上限ではなく電圧の余裕 (4.9V − 回転数分) で頭打ちだったので、
// 斜めのランプは速く (RAMP_DIAG_RATE_V_PER_S) して、速度が出て余裕が減る前に上げ切る
#define RAMP_MAX_V 4.5f
#define RAMP_RATE_V_PER_S 5.0f
#define RAMP_DIAG_RATE_V_PER_S 8.0f
// 滑り始めを見つけたあとも、ピーク (IMU の加速度が一番大きい所) を測るために上げ続ける。
// 滑り始めからこれだけ上げたか、IMU の加速度がピークの RAMP_PEAK_DROP_RATIO 倍を
// RAMP_PEAK_DROP_HOLD_S 下回り続けたら、加速をやめる
#define RAMP_PAST_ONSET_V 0.4f
#define RAMP_PEAK_DROP_RATIO 0.8f
#define RAMP_PEAK_DROP_HOLD_S 0.020f
// 滑り始め: 進む向きの (車輪の加速度 − IMU の加速度) がこれを超えた状態が RAMP_SLIP_HOLD_S 続いた
#define RAMP_SLIP_THRESH_LIN 1.5f   // [m/s^2]
#define RAMP_SLIP_THRESH_ANG 20.0f  // [rad/s^2] (車輪の位置で 1.5m/s² ≒ 20rad/s² × 0.075m)
#define RAMP_SLIP_HOLD_S 0.020f
#define RAMP_DETECT_DELAY_S 0.030f  // ランプを始めてから判定を始めるまで (LPF が落ち着くまで)
#define RAMP_ONSET_HOLD_S 0.030f    // ブレーキ: 滑り始めを見つけてから、トルクを保ってピークを拾う時間
// ブレーキのランプは低い所 (0.8V) から始め、1.4V までは速く (10V/s)、そのあとは 5V/s で上げる。
// (初回の速度別の結果: 開始点が高いと、加速からブレーキに切り替えた一瞬の過渡 (車輪だけ先に減速する) を
//  滑り始めと誤判定して、ブレーキが弱いまま止まった。判定は開始から RAMP_BRAKE_DETECT_DELAY_S 経ってから)
// (2026-09-24 の3回目: 低速 1.0m/s のブレーキは短く、0.8V から 5V/s では限界まで上がる前に止まった (滑らず停止、
//  平均減速度 2.3m/s²)。1.0V から 12V/s で 2.0V まで、そのあと 6V/s に速くした)
#define RAMP_BRAKE_START_V 1.0f
#define RAMP_BRAKE_FAST_UNTIL_V 2.0f
#define RAMP_BRAKE_FAST_RATE 12.0f
#define RAMP_BRAKE_SLOW_RATE 6.0f
// (加速からブレーキへの過渡は 0.06 秒では残った。ブレーキの滑り始めが 1.58V に集中していた → 0.10 秒)
#define RAMP_BRAKE_DETECT_DELAY_S 0.10f
// ブレーキのみの試験: 巡航から直接ブレーキ。ロック判定は当てにならないので、IMU の減速度がピークを過ぎて
// 落ち始めたら、そのときのトルクで保つ (ロックで滑らせない。停止距離を短く、減速度のピークを測る)
#define RAMP_BRAKE_ONLY_MIN_PEAK 1.5f   // ピークがこれ以上になってから、落ち始めを見る [m/s^2]
#define RAMP_BRAKE_ONLY_DROP_HOLD_S 0.030f
// 保つ条件は3つ (どれかが成り立ったら、ピークのトルクで保つ): (1) IMU の減速度の落ち始め (上の 20%/30ms)、
// (2) 滑り始めから RAMP_PAST_ONSET_V 上げた (加速と同じ)、(3) ピークのトルクから RAMP_BRAKE_ONLY_BEYOND_PEAK_V 上げた
// (2026-09-24 の5回目: (1) だけだと、IMU のばたつきで判定が途切れ、4.5V まで上がった本があった)
#define RAMP_BRAKE_ONLY_BEYOND_PEAK_V 0.8f
// ブレーキのみの開始は、通常のブレーキより高く (上がる間の停止距離の延びを減らす。判定は開始から 0.10 秒後)
#define RAMP_BRAKE_ONLY_START_V 1.6f
#define RAMP_BRAKE_ONLY_LIMIT_RATIO 0.70f  // 助走の距離の上限 (予測した長さに対する割合。加速がない分、大きく取る)
#define RAMP_AFTER_SLIP_RATIO 0.9f  // ブレーキで滑り始めを見つけたら、トルク上限をこの割合まで下げる (ロックを止めて、限界の近くで止める)
// 速度別の加速のランプ: 助走 (トルクの差がほぼ 0) から急に 1.8V を掛けると、車輪だけ先に加速する過渡が出て、
// 滑り始めと誤判定した (4本とも 1.9V)。1.0V から速く (10V/s) 上げ、判定は開始から 0.10 秒経ってから
#define RAMP_SPEED_START_V 1.0f
#define RAMP_SPEED_FAST_UNTIL_V 1.8f
#define RAMP_FAST_RATE_V_PER_S 10.0f  // 速度別の加速のランプの、速い側
#define RAMP_SPEED_DETECT_DELAY_S 0.10f
// 加速の目標 (高くして、FF の要求が常にトルク上限を超える状態にする。上限の値 = 実際のトルク)
#define RAMP_TARGET_SPEED 3.0f      // [m/s] (止まった状態から)
#define RAMP_TARGET_ABOVE 2.0f      // [m/s] (巡航の速度 v0 に足す)
#define RAMP_TARGET_ANG 25.0f       // [rad/s]
#define RAMP_CMD_ACCEL 8.0f         // S字の加速度上限 [m/s^2]
#define RAMP_CMD_JERK 1000.0f
#define RAMP_CMD_ANG_ACCEL 80.0f
#define RAMP_CMD_ANG_JERK 2000.0f
// 打ち切り
#define RAMP_SPEED_CAP 1.2f         // [m/s] (止まった状態から)
#define RAMP_SPEED_CAP_ABOVE 0.5f   // [m/s] (巡航の速度 v0 に足す)
#define RAMP_ANG_SPEED_CAP 12.0f    // [rad/s]
#define RAMP_MAX_V_DWELL_S 0.05f    // 上限に達してから打ち切るまで (ピークを拾う)
#define RAMP_BRAKE_TIMEOUT_S 3.0f
#define RAMP_SETTLE_S 0.3f
#define RAMP_RETURN_TIMEOUT_S 4.0f
#define RAMP_PHASE_TIMEOUT_S 4.0f
// 3回目を測るかの判定: 2回の滑り始めの差がこれ以上 (どちらか)。初回の 0.2V/10% は厳しすぎて、
// 10向き中9向きで3回目に入った
#define RAMP_RETRY_DIFF_V 0.30f
#define RAMP_RETRY_DIFF_RATIO 0.15f
// 安全停止の範囲 [m]。低速 (v0=0) の試験だけのときは、従来の狭い範囲 (後ろの空きは約0.5m)
#define RAMP_AREA_X_MIN (-0.4f)
#define RAMP_AREA_X_MAX 1.6f
#define RAMP_AREA_Y_ABS 1.6f
// 速度別の測定のときの範囲 (ユーザー決定: 前 3.5m・後ろ 1.0m・左右 2.5m。原点はスタート位置)
#define RAMP_WIDE_X_MIN (-1.0f)
#define RAMP_WIDE_X_MAX 3.5f
#define RAMP_WIDE_Y_ABS 2.5f
#define RAMP_RANGE_MARGIN 0.3f      // 端から内側へ取る余裕。予測した経路はこの内側に収まるときだけ走らせる
#define RAMP_HEADING_ABORT 0.8f     // 並進の最中の向きのずれ [rad]
// 速度別の測定の助走 (v0 まで巡航): 滑らない範囲でゆっくり加速する
#define RAMP_CRUISE_L 2.0f          // 助走のトルク上限 [V] (前後・斜め。滑り始めの近くまで上げない)
#define RAMP_CRUISE_L_LATERAL 2.3f  // 左右の助走のトルク上限 [V] (左右は同じ加速度に1.3〜1.6倍の力が要り、2.0V では
                                    // 1.0m/s まで 0.4〜0.65m かかった (前後は 0.15〜0.19m))。滑っても助走が遅れるだけ
#define RAMP_CRUISE_ACCEL 2.5f      // 助走の加速度上限 [m/s^2]
#define RAMP_CRUISE_ACCEL_LATERAL_PRED 1.5f  // 距離の予測の既定 (測定が無いとき) に使う、左右の助走の加速度 [m/s^2]
#define RAMP_CRUISE_TOL 0.05f       // v0 に最初に着いたと見なす速度の差 [m/s]
#define RAMP_CRUISE_START_TOL 0.10f // ランプを始めてよいと見なす速度の差 [m/s]
#define RAMP_CRUISE_A_TOL 1.5f      // ランプを始めてよいと見なす加速度 [m/s^2]
#define RAMP_CRUISE_HOLD_S 0.05f
// v0 に着いてから、条件が満たされなくても、この時間が経ったらランプを始める (左右の巡航は速度が揺れて条件が
// 満たされず、ランプを始めないまま範囲を出て安全停止した)
#define RAMP_CRUISE_MAX_DWELL_S 0.25f
// 助走の距離の上限 (予測した1本の長さに対する割合)。v0 に着かないまま超えたら、その1本はやめて止まる
#define RAMP_CRUISE_LIMIT_RATIO 0.55f
#define RAMP_CRUISE_DWELL_S 0.15f   // 距離の予測に足す、v0 に着いてからランプを始めるまでの巡航の時間
// ランプ加速の最長 (滑って速度の打ち切りに届かないときの保険)
#define RAMP_ACCEL_MAX_S 0.6f
// 始点への移動
#define RAMP_GOTO_SPEED 0.8f        // [m/s]
#define RAMP_GOTO_TIMEOUT_S 10.0f
// 必要な長さの予測 (測った値が無いときの既定。測った値があれば速度の2乗などで伸ばす)
#define RAMP_PRED_BRAKE_DECEL 2.6f  // [m/s^2] 保守的なブレーキの減速度
#define RAMP_PRED_MARGIN 1.15f

// 2回目 (機体を90°回して繰り返す) の範囲を1回目から求めるときの、長方形の端から範囲までの内側の距離 [m]
// (機体の半径と、安全停止で止まりきれない分)。1回目の範囲 = 長方形から各端 0.2m 内側、を前提にする
#define RAMP_SESSION_INSET 0.2f

#define RAMP_PAIR_COUNT 5
#define RAMP_SET_MAX 3
#define RAMP_SPEED_LIST_MAX 40
// FF 試験
#define FF_TARGET_SPEED 1.5f        // [m/s]
#define FF_TRACTION_V 3.0f          // FF 試験のトルク上限 [V] (FF の要求より大きく、効かない)
#define FF_WINDOW_MIN_S 0.20f       // これより短い窓は無効
#define FF_PLATEAU_HOLD_S 0.15f     // 指令の加速度が立ち上がってから、平均を取り始めるまで
#define FF_TIMEOUT_S 2.5f
#define FF_PRED_LATERAL_EFF 0.7f    // 距離の予測: 左右は指令の加速度の約 0.7 倍しか出ない (ランプ試験の結果)
#define FF_PRED_BRAKE_DECEL 3.0f    // 距離の予測: FF 試験のあとのブレーキ (通常の設定) の減速度 [m/s^2]
static const float kFfLevels[2] = {1.5f, 2.5f};  // 指令の加速度 [m/s^2]
#define RAMP_AVG_N 100  // 加速度の平均の窓 [制御周期 = 1ms]

static const float kDirVec[8][2] = {
    {1.0f, 0.0f},         {-1.0f, 0.0f},       {0.0f, 1.0f},          {0.0f, -1.0f},
    {0.7071f, 0.7071f},   {-0.7071f, -0.7071f}, {0.7071f, -0.7071f},  {-0.7071f, 0.7071f},
};

typedef enum {
  PH_START = 0,  // 次の1本の準備 (低速の試験)
  PH_ACCEL,
  PH_BRAKE,
  PH_SETTLE,
  PH_RETURN,     // 低速の試験: 組の終わりに原点・向き0へ戻る
  PH_GOTO,       // 速度別の測定: 次の1本の始点へ移動
  PH_CRUISE,     // 速度別の測定: v0 まで巡航
  PH_XFER,       // 1回目が終わったあと、範囲の真ん中へ移動して 90° (右回り) 向きを変える
  PH_FFSTEP,     // FF 試験: PI を切って FF だけで指令の加速度を出す (窓の平均を取る)
  PH_HOME,       // 速度別の測定の終わり: 原点・向き 0 へ戻る
} Phase;

// 滑り始め・ピークの検出 (加速とブレーキで共用)
typedef struct {
  float L;                // 今のトルク上限 [V]
  float L_f;              // IMU と同じ LPF を掛けたトルク上限 (位相をそろえて比べる)
  bool ramping;           // まだ上げているか
  float t;                // この段階の経過時間 [s]
  float at_max_t;         // 上限に達してからの時間 [s]
  float slip_t;           // しきい値を超え続けている時間 [s]
  float cand_L, cand_acc; // しきい値を超え始めたときの値
  bool onset_found;
  float onset_L, onset_acc;
  float after_onset_t;
  float peak_acc, peak_L;
  bool past_onset;        // 加速: 滑り始めのあとも上げ続けてピークを測る (ブレーキは false)
  float drop_t;           // IMU の加速度がピークから落ちている時間 [s]
  float delay_s;          // 始めてから判定を始めるまで [s] (始まりの過渡を避ける)
  float fast_until_v;     // これより下は速く (fast_rate) 上げる (0: 速くしない)
  float fast_rate, slow_rate;  // ランプの速さ [V/s]
  bool brake_only;        // ブレーキのみの試験 (ピークを過ぎたらトルクを保つ)
  bool holding;           // ブレーキのみ: ピークを過ぎたので、トルクを保っている
  // 直近 RAMP_AVG_N ms の加速度の平均 (ピークは、ロックのばたつきなどで実際より大きく出るので、平均も残す)
  float ring[RAMP_AVG_N];
  float ring_sum;
  int ring_pos, ring_n;
  float peak_avg;         // ピークを更新したときの、直近の平均
} Detect;

typedef struct {
  uint8_t dir;
  float v0;
  bool brake_only;         // 巡航のあと、加速せずに直接ブレーキ (停止距離と減速度の測定)
  bool ff;                 // FF 試験 (PI を切って FF だけで加速、指令と実際の加速度の比を測る)
  float a_cmd;             // FF 試験の指令の加速度 [m/s^2]
} SpeedStroke;

static struct {
  bool started;
  uint32_t start_tick;
  Timer dt_timer;
  uint32_t speed_mask;
  bool in_speed;               // 速度別の測定の最中か
  int session;                 // 1: 1回目、2: 機体を 90° 回した2回目 (RAMP_SPEED_ROTATE)
  int set;                     // 0, 1, 2 (低速の試験)
  uint8_t pairs[RAMP_PAIR_COUNT];
  int pair_count;
  int pair_pos;
  int member;                  // 組の1本目 (0) か2本目 (1)
  SpeedStroke sp_list[RAMP_SPEED_LIST_MAX];
  int sp_count;
  int sp_pos;
  float v0;                    // 今の1本の巡航の速度 [m/s] (低速の試験は 0)
  bool brake_only;             // 今の1本は、巡航のあと直接ブレーキ (ブレーキのみの試験)
  int bad_status_n;            // WheelUnit の状態バイトの異常が続いている周期数
  bool ff;                     // 今の1本は FF 試験
  float ff_a_cmd;              // その指令の加速度 [m/s^2]
  float ff_t, ff_plateau_t;    // FF 試験: 経過時間、指令の加速度が立ち上がっている時間 [s]
  bool ff_in_window;
  double ff_sum_cmd, ff_sum_imu, ff_sum_odom;  // 窓の間の合計 (1ms ごと)
  int ff_n;
  float ff_dist;               // FF 試験の間に進行方向へ進んだ距離 [m]
  float tx, ty;                // 始点への移動の目標 [m]
  Phase phase;
  float phase_t;
  Detect det;
  float settle_ok_t;
  float cruise_ok_t;
  float cruise_dist, accel_dist, brake_dist;  // その相の間に進行方向へ進んだ距離 [m]
  bool cruise_reached;         // 助走: v0 に最初に着いたか (着くまでの距離を記録する)
  float cruise_total;          // 助走の間に進んだ距離の合計 [m] (v0 に着いたあとも数える)
  float cruise_limit;          // それの上限 [m] (予測した1本の長さから決める)
  float reach_t;               // v0 に最初に着いてからの時間 [s]
  float pred_len;              // 予測した1本の長さ [m]
  float v_imu;                 // ランプ・ブレーキの間の機体の速度 = ランプの始めの速度 + IMU の加速度の積分 [m/s]
                               // (車輪の空転・ロックの影響を受けない。距離もこれで測る)
  // 旋回の角加速度 (ジャイロと車輪の角速度の微分に LPF)
  float prev_gyro, prev_odom_w, alpha_gyro_f, alpha_odom_f;
  // 床の座標での位置と向き (安全停止と始点への移動に使う)
  float pos_x, pos_y, heading;
  uint8_t log_divider;
  RampStrokeResult* cur;
} rt;

// 速度別の測定の範囲 (既定は RAMP_WIDE_*。RampTest_SetArea で走行ごとに変えられる)
static float area_x_min = RAMP_WIDE_X_MIN, area_x_max = RAMP_WIDE_X_MAX, area_y_abs = RAMP_WIDE_Y_ABS;

void RampTest_SetArea(float x_min, float x_max, float y_abs) {
  // 常識的な範囲に収まらない値は既定に戻す (書き間違いで、広すぎる範囲にならないように)
  bool ok = x_min <= -0.2f && x_min >= -3.0f && x_max >= 0.8f && x_max <= 6.0f && y_abs >= 0.8f && y_abs <= 4.0f;
  area_x_min = ok ? x_min : RAMP_WIDE_X_MIN;
  area_x_max = ok ? x_max : RAMP_WIDE_X_MAX;
  area_y_abs = ok ? y_abs : RAMP_WIDE_Y_ABS;
}

void RampTest_Reset(uint32_t speed_mask) {
  rt.started = false;
  rt.speed_mask = (speed_mask == 0) ? RAMP_SPEED_V0 : speed_mask;
}

void RampTest_Cancel(void) {
  if (rt.started && ramp_result.result == 0) ramp_result.result = 3;
}

static uint16_t U16(float v) {
  if (v < 0.0f) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

static int16_t I16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static int CurrentDir(void) {
  if (rt.in_speed) return rt.sp_list[rt.sp_pos].dir;
  return rt.pairs[rt.pair_pos] * 2 + rt.member;
}
static bool IsRotation(int dir) { return dir >= RAMP_DIR_ROT_CCW; }

static void ResetDetect(float L_start, bool past_onset, float delay_s, float fast_until_v, float fast_rate,
                        float slow_rate) {
  Detect zero = {0};
  rt.det = zero;
  rt.det.L = L_start;
  rt.det.L_f = L_start;
  rt.det.ramping = true;
  rt.det.past_onset = past_onset;
  rt.det.delay_s = delay_s;
  rt.det.fast_until_v = fast_until_v;
  rt.det.fast_rate = fast_rate;
  rt.det.slow_rate = slow_rate;
}

// 電圧上限までの余裕 (4輪の中で一番小さいもの) [V]
static float MinHeadroom(const OmniDrive* od) {
  float m = WHEEL_VOLT_MAX;
  for (int i = 0; i < 4; i++) {
    float h = WHEEL_VOLT_MAX - fabsf(WheelVoltage_Feedforward(i, od->vel_wheel_angular[i]));
    if (h < m) m = h;
  }
  return m;
}

// 1周期ぶんの検出。a_imu・a_odom は進む向き (ブレーキなら減速の向き) を正にした値。
// valid が false の間は判定しない (LPF が落ち着く前、出力を縮めていないとき)
static void DetectStep(float dt, float a_imu, float a_odom, float thresh, bool valid) {
  Detect* d = &rt.det;
  d->t += dt;
  const float tau = 1.0f / (2.0f * (float)PI * TCS_ACCEL_LPF_HZ);
  d->L_f += dt / (tau + dt) * (d->L - d->L_f);
  // 直近の平均 (1ms ごとに1つずつ積む)
  d->ring_sum += a_imu - d->ring[d->ring_pos];
  d->ring[d->ring_pos] = a_imu;
  d->ring_pos = (d->ring_pos + 1) % RAMP_AVG_N;
  if (d->ring_n < RAMP_AVG_N) d->ring_n++;
  if (d->onset_found) {
    // 見つけた後もピークを拾う (加速は上げ続けながら、ブレーキは保つ間だけ)。
    // 保つ時間は valid に関係なく進める (止まらずにトルクを下げられるように)
    d->after_onset_t += dt;
    bool track = d->past_onset || d->after_onset_t <= RAMP_ONSET_HOLD_S;
    if (valid && track && a_imu > d->peak_acc) {
      d->peak_acc = a_imu;
      d->peak_L = d->L_f;
      d->peak_avg = d->ring_sum / (float)d->ring_n;
    }
    if (d->past_onset && valid) {
      d->drop_t = (a_imu < RAMP_PEAK_DROP_RATIO * d->peak_acc) ? d->drop_t + dt : 0.0f;
    }
    return;
  }
  if (!valid || d->t < d->delay_s) {
    d->slip_t = 0.0f;
    return;
  }
  if (a_imu > d->peak_acc) {
    d->peak_acc = a_imu;
    d->peak_L = d->L_f;
    d->peak_avg = d->ring_sum / (float)d->ring_n;
  }
  if (d->brake_only) {
    d->drop_t = (d->peak_acc > RAMP_BRAKE_ONLY_MIN_PEAK && a_imu < RAMP_PEAK_DROP_RATIO * d->peak_acc)
                    ? d->drop_t + dt
                    : 0.0f;
  }
  if (a_odom - a_imu > thresh) {
    if (d->slip_t == 0.0f) {
      d->cand_L = d->L_f;
      d->cand_acc = a_imu;
    }
    d->slip_t += dt;
    if (d->slip_t >= RAMP_SLIP_HOLD_S) {
      d->onset_found = true;
      d->onset_L = d->cand_L;
      d->onset_acc = d->cand_acc;
      // 加速 (とブレーキのみ) は、滑り始めのあともランプを続ける (ピークを測るため)。
      // 通常のブレーキは止めて、保つ・下げるに移る
      if (!d->past_onset) d->ramping = false;
    }
  } else {
    d->slip_t = 0.0f;
  }
}

static void AdvanceRamp(float dt) {
  Detect* d = &rt.det;
  if (d->onset_found && !d->past_onset) {
    // ブレーキ: 滑り始めを見つけたら、ピークを拾う間だけ保ち、その後は下げてロックを止める
    if (d->after_onset_t >= RAMP_ONSET_HOLD_S) d->L = RAMP_AFTER_SLIP_RATIO * d->onset_L;
    return;
  }
  if (!d->ramping) return;
  d->L += ((d->L < d->fast_until_v) ? d->fast_rate : d->slow_rate) * dt;
  if (d->L >= RAMP_MAX_V) {
    d->L = RAMP_MAX_V;
    d->at_max_t += dt;
  }
}

static void SaveDetect(RampPhaseResult* r, RampEndReason reason, float speed, float acc_scale) {
  const Detect* d = &rt.det;
  r->onset_v_x100 = d->onset_found ? U16(d->onset_L * 100.0f) : 0;
  r->peak_v_x100 = U16(d->peak_L * 100.0f);
  r->peak_acc = I16(d->peak_acc * acc_scale);
  r->onset_acc = d->onset_found ? I16(d->onset_acc * acc_scale) : 0;
  r->avg_acc = I16(d->peak_avg * acc_scale);
  r->speed = I16(speed * 1000.0f);
  r->end_reason = (uint16_t)reason;
}

// volt_tune をランプ用にする (トルク上限だけランプの値、S字は十分速く)。基準は、既定値＋開始の指示の上書き
static void SetRampTune(float L) {
  VoltTune_LoadBase(&volt_tune);
  volt_tune.traction_limit_v = L;  // Sanitize は通さない (試験の中だけ 3.2V を超えてよい)
  volt_tune.max_accel = RAMP_CMD_ACCEL;
  volt_tune.max_jerk = RAMP_CMD_JERK;
  volt_tune.max_ang_accel = RAMP_CMD_ANG_ACCEL;
  volt_tune.max_ang_jerk = RAMP_CMD_ANG_JERK;
}

// 助走用: 滑らない範囲でゆっくり加速する (左右は、力が多く要るので、トルク上限を少し上げる)
static void SetCruiseTune(int dir) {
  VoltTune_LoadBase(&volt_tune);
  volt_tune.traction_limit_v = (dir == RAMP_DIR_LEFT || dir == RAMP_DIR_RIGHT) ? RAMP_CRUISE_L_LATERAL : RAMP_CRUISE_L;
  volt_tune.max_accel = RAMP_CRUISE_ACCEL;
}

static void Drive(OmniDrive* od, const Imu* imu, float vx, float vy, float omega) {
  OmniDrive_SetControlMode(od, true);
  od->tcs.config.enable_s_curve = true;
  OmniDrive_SetVelEx(od, (int16_t)Constrain(vx * 1000.0f, -32000.0f, 32000.0f),
                     (int16_t)Constrain(vy * 1000.0f, -32000.0f, 32000.0f),
                     (int16_t)Constrain(omega * 1000.0f, -32000.0f, 32000.0f), imu);
}

// 向きを heading_target に保つ角速度の指令 (動作パターンのテストと同じ PD)
static float HeadingHold(const Robot* robot, float heading_target) {
  float eh = heading_target - rt.heading;
  float ang = fminf(6.0f, fminf(sqrtf(2.0f * 20.0f * fabsf(eh)), 8.0f * fabsf(eh)));
  float omega = (eh >= 0.0f) ? ang : -ang;
  omega -= 0.5f * robot->imu.yaw_rate;
  return Constrain(omega, -6.0f, 6.0f);
}

// S字の指令を今の実際の速度にそろえる (加速からブレーキに移るとき、指令が実機より先に行って
// いるとブレーキのはずが加速し続けるため)
static void SeedCommand(OmniDrive* od, const Robot* robot) {
  od->tcs.current_vx = od->tcs.odom_vx;
  od->tcs.current_vy = od->tcs.odom_vy;
  od->tcs.current_omega = robot->imu.yaw_rate;
  od->tcs.current_ax = 0.0f;
  od->tcs.current_ay = 0.0f;
  od->tcs.current_alpha = 0.0f;
}

// 低速 (v0=0) の向き dir の set 回目の加速の滑り始め [V] (見つからなかったとき・記録が無いときは −1)
static float OnsetVOf(int dir, int set) {
  for (int i = 0; i < ramp_result.stroke_count; i++) {
    const RampStrokeResult* s = &ramp_result.stroke[i];
    if (s->v0_x100 == 0 && s->dir == dir && s->set == set + 1) {
      return (s->accel.onset_v_x100 > 0) ? s->accel.onset_v_x100 * 0.01f : -1.0f;
    }
  }
  return -1.0f;
}

static bool IsFar(float a, float b) {
  if (a < 0.0f || b < 0.0f) return false;
  float diff = fabsf(a - b);
  return diff >= RAMP_RETRY_DIFF_V || diff >= RAMP_RETRY_DIFF_RATIO * fmaxf(a, b);
}

// 1回目と2回目の滑り始めの差が大きい向きの組を選ぶ (滑り始めが見つからなかった回は比べない)。無ければ false
static bool PlanRetry(void) {
  rt.pair_count = 0;
  for (int p = 0; p < RAMP_PAIR_COUNT; p++) {
    bool far = false;
    for (int m = 0; m < 2; m++) {
      int dir = p * 2 + m;
      if (IsFar(OnsetVOf(dir, 0), OnsetVOf(dir, 1))) far = true;
    }
    if (far) {
      rt.pairs[rt.pair_count++] = (uint8_t)p;
      ramp_result.retry_mask |= (uint16_t)(3U << (p * 2));
    }
  }
  return rt.pair_count > 0;
}

// 3回目を測った向きで、3回の最大と最小の差がまだ大きいものを「不安定」とする
static void MarkUnstable(void) {
  for (int dir = 0; dir < RAMP_DIR_COUNT; dir++) {
    if (!(ramp_result.retry_mask & (1U << dir))) continue;
    float lo = 1e9f, hi = -1e9f;
    for (int set = 0; set < RAMP_SET_MAX; set++) {
      float v = OnsetVOf(dir, set);
      if (v < 0.0f) continue;
      lo = fminf(lo, v);
      hi = fmaxf(hi, v);
    }
    if (hi >= lo && IsFar(lo, hi)) ramp_result.unstable_mask |= (uint16_t)(1U << dir);
  }
}

static RampTestStatus Abort(OmniDrive* od, uint16_t reason) {
  OmniDrive_SetFree(od);
  VoltTune_SetDefaults(&volt_tune);
  ramp_result.result = 2;
  ramp_result.abort_reason = reason;
  printf("# ramp test aborted: reason=%u stroke=%u x=%d y=%d mm th=%d mrad\n", reason,
         ramp_result.stroke_count, (int)(rt.pos_x * 1000.0f), (int)(rt.pos_y * 1000.0f),
         (int)(rt.heading * 1000.0f));
  return RAMP_ABORTED;
}

static RampTestStatus Finish(OmniDrive* od) {
  OmniDrive_SetFree(od);
  VoltTune_SetDefaults(&volt_tune);
  ramp_result.result = 1;
  printf("# ramp test finished: strokes=%u retry=0x%03x unstable=0x%03x\n",
         ramp_result.stroke_count, ramp_result.retry_mask, ramp_result.unstable_mask);
  return RAMP_FINISHED;
}

// ---- 速度別の測定 ----

// 向き dir・巡航の速度 v0 の1本 (助走・ランプ・ブレーキ) に要る長さを予測する [m]。
// 同じ向きで、より低い速度の測定が済んでいれば、その距離を伸ばす (助走とブレーキは速度の2乗、
// ランプは (v0+0.2) に比例)。無ければ既定の式
static float PredictLength(int dir, float v0, bool brake_only) {
  const RampStrokeResult* best = NULL;
  for (int i = 0; i < ramp_result.stroke_count; i++) {
    const RampStrokeResult* s = &ramp_result.stroke[i];
    if (s->dir != dir || s->v0_x100 == 0 || s->brake_dist_mm == 0) continue;
    if (s->v0_x100 * 0.01f >= v0) continue;
    if (best == NULL || s->v0_x100 > best->v0_x100) best = s;
  }
  float cruise, ramp, brake;
  float v_b = brake_only ? v0 : v0 + RAMP_SPEED_CAP_ABOVE;
  if (best != NULL) {
    float vm = best->v0_x100 * 0.01f;
    float vbm = fmaxf(best->v_brake_start * 0.001f, 0.5f);
    cruise = best->cruise_dist_mm * 0.001f * (v0 * v0) / (vm * vm);
    ramp = brake_only ? 0.0f : best->accel_dist_mm * 0.001f * (v0 + 0.2f) / (vm + 0.2f);
    brake = best->brake_dist_mm * 0.001f * (v_b * v_b) / (vbm * vbm);
  } else {
    const bool lateral = (dir == RAMP_DIR_LEFT || dir == RAMP_DIR_RIGHT);
    cruise = v0 * v0 / (2.0f * (lateral ? RAMP_CRUISE_ACCEL_LATERAL_PRED : RAMP_CRUISE_ACCEL));
    ramp = brake_only ? 0.0f : 0.2f * (v0 + 0.2f);
    brake = v_b * v_b / (2.0f * RAMP_PRED_BRAKE_DECEL);
  }
  return RAMP_PRED_MARGIN * (cruise + v0 * RAMP_CRUISE_DWELL_S + ramp + brake);
}

// FF 試験の1本に要る長さの予測 [m]: 加速 (v²/(2a)、左右は指令の約 0.7 倍しか出ない) + ブレーキ
static float PredictFfLength(int dir, float a_cmd) {
  const bool lateral = (dir == RAMP_DIR_LEFT || dir == RAMP_DIR_RIGHT);
  const float a_eff = a_cmd * (lateral ? FF_PRED_LATERAL_EFF : 1.0f);
  const float v = FF_TARGET_SPEED;
  return RAMP_PRED_MARGIN * (v * v / (2.0f * a_eff) + v * v / (2.0f * FF_PRED_BRAKE_DECEL) + 0.1f);
}

// 速度の指定から、走らせる1本の一覧を作る (速度の低い順)
static void BuildSpeedList(void) {
  static const struct {
    float v0;
    int first;   // 最初の向き (0: 前後左右の前、2: 左右の左、4: 斜め)
    int count;   // 向きの数
    uint32_t bit;
    bool brake_only;
  } kLevels[] = {
      {1.0f, 0, 4, RAMP_SPEED_AXIS_1_0, false}, {1.5f, 4, 4, RAMP_SPEED_DIAG_1_5, false},
      {2.0f, 0, 4, RAMP_SPEED_AXIS_2_0, false}, {2.0f, 4, 4, RAMP_SPEED_DIAG_2_0, false},
      {3.0f, 0, 4, RAMP_SPEED_AXIS_3_0, false},
      // ブレーキのみ (加速のランプ無し)。速度の低い順に (低い方の結果で、高い方の必要な長さを予測する)。
      // 前後 (b2, b2.5) と、左右 (b2l。このエリア 3.5×2.5m では、助走が長く走れない) を別の指定にした
      {2.0f, 0, 2, RAMP_SPEED_BRAKE_2_0, true}, {2.5f, 0, 2, RAMP_SPEED_BRAKE_2_5, true},
      {2.0f, 2, 2, RAMP_SPEED_BRAKE_2_0L, true},
  };
  rt.sp_count = 0;
  for (unsigned i = 0; i < sizeof(kLevels) / sizeof(kLevels[0]); i++) {
    if (!(rt.speed_mask & kLevels[i].bit)) continue;
    const int first = kLevels[i].first;
    for (int k = 0; k < kLevels[i].count && rt.sp_count < RAMP_SPEED_LIST_MAX; k++) {
      rt.sp_list[rt.sp_count].dir = (uint8_t)(first + k);
      rt.sp_list[rt.sp_count].v0 = kLevels[i].v0;
      rt.sp_list[rt.sp_count].brake_only = kLevels[i].brake_only;
      rt.sp_list[rt.sp_count].ff = false;
      rt.sp_list[rt.sp_count].a_cmd = 0.0f;
      rt.sp_count++;
    }
  }
  // FF 試験の軽量版: 前・後・左・右 × 指令の加速度 2.5m/s² × 2回
  if (rt.speed_mask & RAMP_SPEED_FF_FAST) {
    for (int rep = 0; rep < 2; rep++) {
      for (int dir = 0; dir < 4; dir++) {
        if (rt.sp_count >= RAMP_SPEED_LIST_MAX) break;
        SpeedStroke* sp = &rt.sp_list[rt.sp_count++];
        sp->dir = (uint8_t)dir;
        sp->v0 = FF_TARGET_SPEED;
        sp->brake_only = false;
        sp->ff = true;
        sp->a_cmd = kFfLevels[1];
      }
    }
  }
  // FF 試験: 前・後・左・右 × 指令の加速度 2段階 × 2回 (前後は比較の基準)
  if (rt.speed_mask & RAMP_SPEED_FF) {
    for (int rep = 0; rep < 2; rep++) {
      for (int dir = 0; dir < 4; dir++) {
        for (int lv = 0; lv < 2; lv++) {
          if (rt.sp_count >= RAMP_SPEED_LIST_MAX) break;
          SpeedStroke* sp = &rt.sp_list[rt.sp_count++];
          sp->dir = (uint8_t)dir;
          sp->v0 = FF_TARGET_SPEED;
          sp->brake_only = false;
          sp->ff = true;
          sp->a_cmd = kFfLevels[lv];
        }
      }
    }
  }
}

// 予測した長さの経路が、範囲の端から余裕を取った内側に収まるか。収まるなら始点を返す
// (範囲の中心を、経路の真ん中にする。+向きと −向きの組は、同じ経路を行き来する)
static bool PlanPath(int dir, float length, float* start_x, float* start_y) {
  const float cx = 0.5f * (area_x_min + area_x_max);
  const float cy = 0.0f;
  float ux = kDirVec[dir][0], uy = kDirVec[dir][1];
  float x0 = cx - ux * 0.5f * length, y0 = cy - uy * 0.5f * length;
  float x1 = x0 + ux * length, y1 = y0 + uy * length;
  const float xlo = area_x_min + RAMP_RANGE_MARGIN, xhi = area_x_max - RAMP_RANGE_MARGIN;
  const float ylim = area_y_abs - RAMP_RANGE_MARGIN;
  bool ok = x0 >= xlo && x0 <= xhi && x1 >= xlo && x1 <= xhi && fabsf(y0) <= ylim && fabsf(y1) <= ylim;
  *start_x = x0;
  *start_y = y0;
  return ok;
}

// 速度別の一覧の rt.sp_pos 以降で、走らせられる最初の1本を選んで始点への移動に入る。走らせない1本は
// 「範囲不足」と記録して飛ばす。もう無ければ false
static bool PrepareNextSpeedStroke(uint32_t elapsed_ms) {
  while (rt.sp_pos < rt.sp_count) {
    int dir = rt.sp_list[rt.sp_pos].dir;
    float v0 = rt.sp_list[rt.sp_pos].v0;
    bool brake_only = rt.sp_list[rt.sp_pos].brake_only;
    const bool ff = rt.sp_list[rt.sp_pos].ff;
    const float a_cmd = rt.sp_list[rt.sp_pos].a_cmd;
    float length = ff ? PredictFfLength(dir, a_cmd) : PredictLength(dir, v0, brake_only);
    float sx, sy;
    if (PlanPath(dir, length, &sx, &sy)) {
      rt.v0 = v0;
      rt.brake_only = brake_only;
      rt.ff = ff;
      rt.ff_a_cmd = a_cmd;
      rt.pred_len = length;
      rt.cruise_limit = fmaxf(0.5f, (brake_only ? RAMP_BRAKE_ONLY_LIMIT_RATIO : RAMP_CRUISE_LIMIT_RATIO) * length);
      rt.tx = sx;
      rt.ty = sy;
      rt.phase = PH_GOTO;
      rt.phase_t = 0.0f;
      rt.settle_ok_t = 0.0f;
      return true;
    }
    if (ff) {
      // FF 試験の1本は、走らずに「範囲不足」と記録する
      if (ff_result_count < FF_RESULT_MAX) {
        FfStepResult* f = &ff_results[ff_result_count++];
        FfStepResult zf = {0};
        *f = zf;
        f->dir = (uint8_t)dir;
        f->session = (uint8_t)rt.session;
        f->a_nom_x100 = U16(a_cmd * 100.0f);
        f->v_end_mmps = I16(length * 1000.0f);  // 予測した必要な長さ [mm] (走らなかったときだけ)
      }
    } else if (ramp_result.stroke_count < RAMP_MAX_STROKES) {
      RampStrokeResult* s = &ramp_result.stroke[ramp_result.stroke_count++];
      RampStrokeResult zero = {0};
      *s = zero;
      s->dir = (uint8_t)dir;
      s->set = (uint8_t)rt.session;
      s->batt = 255;
      s->t_start_ms = (uint16_t)(elapsed_ms > 65535U ? 65535U : elapsed_ms);
      s->v0_x100 = U16(v0 * 100.0f);
      s->accel.end_reason = RAMP_END_RANGE;
      s->cruise_dist_mm = U16(length * 1000.0f);  // 予測した必要な長さ
    }
    printf("# ramp test: skipped dir=%d v0=%d (need %d mm, range short)\n", dir, (int)(v0 * 100.0f),
           (int)(length * 1000.0f));
    rt.sp_pos++;
  }
  return false;
}

// 速度別の測定を始める。走らせる1本が無ければ false
static bool BeginSpeedSection(uint32_t elapsed_ms) {
  BuildSpeedList();
  rt.in_speed = true;
  rt.sp_pos = 0;
  // 波形のログは、速度別の測定のぶんを新しく取る (低速の試験のぶんは結果の表で足りる)
  TcsLog_Reset();
  return PrepareNextSpeedStroke(elapsed_ms);
}

static void NewStrokeRecord(uint32_t elapsed_ms, int dir, int set) {
  rt.cur = NULL;
  if (ramp_result.stroke_count < RAMP_MAX_STROKES) {
    rt.cur = &ramp_result.stroke[ramp_result.stroke_count++];
    RampStrokeResult zero = {0};
    *rt.cur = zero;
    rt.cur->dir = (uint8_t)dir;
    rt.cur->set = (uint8_t)set;
    rt.cur->batt = 255;
    rt.cur->t_start_ms = (uint16_t)(elapsed_ms > 65535U ? 65535U : elapsed_ms);
    rt.cur->v0_x100 = U16(rt.v0 * 100.0f);
  }
  rt.cruise_dist = rt.accel_dist = rt.brake_dist = 0.0f;
  rt.cruise_reached = false;
  rt.cruise_total = 0.0f;
  rt.reach_t = 0.0f;
}

static void BeginRamp(OmniDrive* od, const Robot* robot, float start_speed) {
  rt.v_imu = fmaxf(start_speed, 0.0f);
  for (int k = 0; k < 3; k++) od->vel_fb_integral[k] = 0.0f;
  SeedCommand(od, robot);
  const int cur_dir = CurrentDir();
  const float rate = (cur_dir >= 4 && cur_dir < 8) ? RAMP_DIAG_RATE_V_PER_S : RAMP_RATE_V_PER_S;
  if (rt.v0 > 0.05f) {
    ResetDetect(RAMP_SPEED_START_V, true, RAMP_SPEED_DETECT_DELAY_S, RAMP_SPEED_FAST_UNTIL_V,
                RAMP_FAST_RATE_V_PER_S, rate);
  } else {
    ResetDetect(RAMP_START_V, true, RAMP_DETECT_DELAY_S, 0.0f, rate, rate);
  }
  float odom_vx, odom_vy, odom_w;
  OmniDrive_GetVelF(od, &odom_vx, &odom_vy, &odom_w);
  rt.prev_gyro = robot->imu.yaw_rate;
  rt.prev_odom_w = odom_w;
  rt.alpha_gyro_f = 0.0f;
  rt.alpha_odom_f = 0.0f;
  rt.phase = PH_ACCEL;
  rt.phase_t = 0.0f;
}

// ブレーキのみ: 巡航から、加速せずに直接ブレーキに入る (start_speed は巡航の速度)
static void BeginBrakeOnly(OmniDrive* od, const Robot* robot, float start_speed) {
  rt.v_imu = fmaxf(start_speed, 0.0f);
  for (int k = 0; k < 3; k++) od->vel_fb_integral[k] = 0.0f;
  SeedCommand(od, robot);
  if (rt.cur != NULL) rt.cur->v_brake_start = U16(fmaxf(start_speed, 0.0f) * 1000.0f);
  ResetDetect(RAMP_BRAKE_ONLY_START_V, true, RAMP_BRAKE_DETECT_DELAY_S, RAMP_BRAKE_FAST_UNTIL_V,
              RAMP_BRAKE_FAST_RATE, RAMP_BRAKE_SLOW_RATE);
  rt.det.brake_only = true;
  rt.phase = PH_BRAKE;
  rt.phase_t = 0.0f;
}

// 始点 (tx, ty)・向き0へ低速で移動する。着いたら true (オドメトリのずれが積み重ならないようにするため)
static bool GotoStep(OmniDrive* od, const Robot* robot, float tx, float ty, float target_heading, float dt,
                     float c, float s) {
  VoltTune_LoadBase(&volt_tune);
  float ex = tx - rt.pos_x, ey = ty - rt.pos_y;
  float dist = sqrtf(ex * ex + ey * ey);
  float sp = fminf(RAMP_GOTO_SPEED, fminf(sqrtf(2.0f * 3.0f * dist), 4.0f * dist));
  float vxw = (dist > 1e-3f) ? ex / dist * sp : 0.0f;
  float vyw = (dist > 1e-3f) ? ey / dist * sp : 0.0f;
  Drive(od, &robot->imu, vxw * c + vyw * s, -vxw * s + vyw * c, HeadingHold(robot, target_heading));
  if (dist < 0.03f && fabsf(rt.heading - target_heading) < 0.05f) {
    rt.settle_ok_t += dt;
  } else {
    rt.settle_ok_t = 0.0f;
  }
  return rt.settle_ok_t >= 0.15f;
}

RampTestStatus RampTest_Step(Robot* robot) {
  OmniDrive* od = &robot->omni_drive;

  // キック・ドリブルは使わない (動作パターンのテストと同じ)
  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  if (!rt.started) {
    rt.started = true;
    if (rt.speed_mask == 0) rt.speed_mask = RAMP_SPEED_V0;
    rt.start_tick = HAL_GetTick();
    Timer_Init(&rt.dt_timer);
    Timer_Reset(&rt.dt_timer);
    uint32_t seq = ramp_result.seq + 1;
    RampRunResult zero = {0};
    ramp_result = zero;
    ramp_result.seq = seq;
    ramp_result.speed_mask = rt.speed_mask;
    rt.in_speed = false;
    rt.session = 1;
    rt.set = 0;
    rt.pair_count = RAMP_PAIR_COUNT;
    for (int p = 0; p < RAMP_PAIR_COUNT; p++) rt.pairs[p] = (uint8_t)p;
    rt.pair_pos = 0;
    rt.member = 0;
    rt.v0 = 0.0f;
    rt.brake_only = false;
    rt.ff = false;
    rt.bad_status_n = 0;
    ff_result_count = 0;
    ramp_abort_status = 0;
    ramp_abort_batt = 0;
    rt.pos_x = rt.pos_y = rt.heading = 0.0f;
    rt.phase = PH_START;
    rt.phase_t = 0.0f;
    rt.log_divider = 0;
    TCS_Reset(&od->tcs);
    TcsLog_Reset();
    printf("# ramp test started (seq=%lu speed_mask=0x%02lx area x[%d,%d] y+-%d mm)\n", (unsigned long)seq,
           (unsigned long)rt.speed_mask, (int)(area_x_min * 1000.0f), (int)(area_x_max * 1000.0f),
           (int)(area_y_abs * 1000.0f));
    // 低速の試験を含まないときは、速度別の測定から始める
    if (!(rt.speed_mask & RAMP_SPEED_V0)) {
      if (!BeginSpeedSection(0)) return Finish(od);
    }
  }

  float dt = Timer_Read(&rt.dt_timer);
  Timer_Reset(&rt.dt_timer);
  if (dt <= 0.0f || dt > 0.05f) dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;
  uint32_t elapsed_ms = HAL_GetTick() - rt.start_tick;
  rt.phase_t += dt;

  // 位置と向き (機体座標のオドメトリを床の座標に直して積分。動作パターンのテストと同じ)
  rt.heading += robot->imu.yaw_rate * dt;
  float c = cosf(rt.heading), s = sinf(rt.heading);
  rt.pos_x += (od->tcs.odom_vx * c - od->tcs.odom_vy * s) * dt;
  rt.pos_y += (od->tcs.odom_vx * s + od->tcs.odom_vy * c) * dt;

  // 安全停止
  uint8_t st = od->wheel_status[0] | od->wheel_status[1] | od->wheel_status[2] | od->wheel_status[3];
  if (rt.cur != NULL) {
    rt.cur->status_or |= st;
    uint8_t batt = robot->info.battery_voltage;
    if (batt > 0 && batt < rt.cur->batt) rt.cur->batt = batt;
  }
  const bool wide = (rt.speed_mask & ~RAMP_SPEED_V0) != 0U;
  const float x_min = wide ? area_x_min : RAMP_AREA_X_MIN;
  const float x_max = wide ? area_x_max : RAMP_AREA_X_MAX;
  const float y_abs = wide ? area_y_abs : RAMP_AREA_Y_ABS;
  if (rt.pos_x < x_min || rt.pos_x > x_max || fabsf(rt.pos_y) > y_abs) return Abort(od, 1);
  int dir = CurrentDir();
  bool translating = (rt.phase == PH_ACCEL || rt.phase == PH_BRAKE || rt.phase == PH_CRUISE ||
                      rt.phase == PH_FFSTEP) &&
                     !IsRotation(dir);
  if (translating && fabsf(rt.heading) > RAMP_HEADING_ABORT) return Abort(od, 2);
  bool moving_phase = (rt.phase == PH_RETURN || rt.phase == PH_GOTO || rt.phase == PH_XFER || rt.phase == PH_HOME);
  if (!moving_phase && rt.phase_t > RAMP_PHASE_TIMEOUT_S) return Abort(od, 3);
  if ((rt.phase == PH_GOTO || rt.phase == PH_XFER || rt.phase == PH_HOME) && rt.phase_t > RAMP_GOTO_TIMEOUT_S) {
    return Abort(od, 3);
  }
  // bit1: 電源電圧範囲外、bit2: 過熱。RAMP_STATUS_DEBOUNCE_CYCLES 続いたときだけ止める。止めたときの状態を残す
  if (st & 0x06U) {
    if (++rt.bad_status_n >= RAMP_STATUS_DEBOUNCE_CYCLES) {
      ramp_abort_status = (uint32_t)od->wheel_status[0] | ((uint32_t)od->wheel_status[1] << 8) |
                          ((uint32_t)od->wheel_status[2] << 16) | ((uint32_t)od->wheel_status[3] << 24);
      ramp_abort_batt = robot->info.battery_voltage;
      return Abort(od, 4);
    }
  } else {
    rt.bad_status_n = 0;
  }

  DigitalOut_Write(&robot->led0, 1);

  // 加速度 (進む向きを正にする)
  float a_imu = 0.0f, a_odom = 0.0f, speed = 0.0f, thresh = RAMP_SLIP_THRESH_LIN, acc_scale = 100.0f;
  if (IsRotation(dir)) {
    float sgn = (dir == RAMP_DIR_ROT_CCW) ? 1.0f : -1.0f;
    float odom_vx, odom_vy, odom_w;
    OmniDrive_GetVelF(od, &odom_vx, &odom_vy, &odom_w);
    const float tau = 1.0f / (2.0f * (float)PI * TCS_ACCEL_LPF_HZ);
    const float k = dt / (tau + dt);
    rt.alpha_gyro_f += k * ((robot->imu.yaw_rate - rt.prev_gyro) / dt - rt.alpha_gyro_f);
    rt.alpha_odom_f += k * ((odom_w - rt.prev_odom_w) / dt - rt.alpha_odom_f);
    rt.prev_gyro = robot->imu.yaw_rate;
    rt.prev_odom_w = odom_w;
    a_imu = sgn * rt.alpha_gyro_f;
    a_odom = sgn * rt.alpha_odom_f;
    speed = sgn * robot->imu.yaw_rate;
    thresh = RAMP_SLIP_THRESH_ANG;
    acc_scale = 10.0f;
  } else if (dir < 8) {
    float ux = kDirVec[dir][0], uy = kDirVec[dir][1];
    a_imu = od->tcs.a_imu_x * ux + od->tcs.a_imu_y * uy;
    a_odom = od->tcs.a_odom_x * ux + od->tcs.a_odom_y * uy;
    speed = od->tcs.odom_vx * ux + od->tcs.odom_vy * uy;
  }

  switch (rt.phase) {
    case PH_START:
      rt.brake_only = false;
      rt.ff = false;
      NewStrokeRecord(elapsed_ms, dir, rt.set + 1);
      BeginRamp(od, robot, 0.0f);
      return RAMP_RUNNING;

    case PH_GOTO:
      if (GotoStep(od, robot, rt.tx, rt.ty, 0.0f, dt, c, s)) {
        SeedCommand(od, robot);
        for (int k = 0; k < 3; k++) od->vel_fb_integral[k] = 0.0f;
        rt.cruise_ok_t = 0.0f;
        if (rt.ff) {
          // FF 試験: ランプの結果 (ramp_result) には記録しない。ff_results に、窓の平均を記録する
          rt.cur = NULL;
          rt.ff_t = rt.ff_plateau_t = 0.0f;
          rt.ff_in_window = false;
          rt.ff_sum_cmd = rt.ff_sum_imu = rt.ff_sum_odom = 0.0;
          rt.ff_n = 0;
          rt.ff_dist = 0.0f;
          rt.pred_len = PredictFfLength(dir, rt.ff_a_cmd);
          rt.phase = PH_FFSTEP;
        } else {
          NewStrokeRecord(elapsed_ms, dir, rt.session);
          rt.phase = PH_CRUISE;
        }
        rt.phase_t = 0.0f;
      } else if (rt.phase_t > RAMP_GOTO_TIMEOUT_S - 0.5f) {
        return Abort(od, 3);  // 始点に着けなかった (位置が当てにならない)
      }
      break;

    case PH_FFSTEP: {
      // FF 試験: PI を切って (kp・ki = 0)、FF だけで、指令の加速度 (S字の max_accel) を出す。
      // 指令の加速度が立ち上がってから 0.15 秒後〜落ち始めるまでの窓で、実際の加速度 (IMU) の平均を取る
      VoltTune_LoadBase(&volt_tune);
      volt_tune.traction_limit_v = FF_TRACTION_V;
      volt_tune.max_accel = rt.ff_a_cmd;
      volt_tune.kp_lin = 0.0f;
      volt_tune.ki_lin = 0.0f;
      const float ux = kDirVec[dir][0], uy = kDirVec[dir][1];
      Drive(od, &robot->imu, ux * rt.v0, uy * rt.v0, HeadingHold(robot, 0.0f));
      rt.ff_t += dt;
      rt.ff_dist += fmaxf(speed, 0.0f) * dt;
      const float a_cmd_now = od->tcs.current_ax * ux + od->tcs.current_ay * uy;
      bool finished = false;
      if (!rt.ff_in_window) {
        if (a_cmd_now >= 0.95f * rt.ff_a_cmd) {
          rt.ff_plateau_t += dt;
          if (rt.ff_plateau_t >= FF_PLATEAU_HOLD_S) rt.ff_in_window = true;
        } else {
          rt.ff_plateau_t = 0.0f;
        }
      } else if (a_cmd_now < 0.90f * rt.ff_a_cmd) {
        finished = true;  // 指令の加速度が落ち始めた (目標速度に近づいた)
      } else {
        rt.ff_sum_cmd += a_cmd_now;
        rt.ff_sum_imu += a_imu;
        rt.ff_sum_odom += a_odom;
        rt.ff_n++;
      }
      if (rt.ff_t > FF_TIMEOUT_S || rt.ff_dist > 0.9f * rt.pred_len) finished = true;  // 届かない・長すぎる
      if (finished) {
        if (ff_result_count < FF_RESULT_MAX) {
          FfStepResult* f = &ff_results[ff_result_count++];
          FfStepResult zf = {0};
          *f = zf;
          f->dir = (uint8_t)dir;
          f->session = (uint8_t)rt.session;
          f->a_nom_x100 = U16(rt.ff_a_cmd * 100.0f);
          if (rt.ff_n > 0) {
            f->a_cmd_x100 = I16((float)(rt.ff_sum_cmd / rt.ff_n) * 100.0f);
            f->a_imu_x100 = I16((float)(rt.ff_sum_imu / rt.ff_n) * 100.0f);
            f->a_odom_x100 = I16((float)(rt.ff_sum_odom / rt.ff_n) * 100.0f);
          }
          f->win_ms = (uint16_t)(rt.ff_n > 65535 ? 65535 : rt.ff_n);
          f->v_end_mmps = I16(speed * 1000.0f);
          f->status_or = st;
          f->valid = (rt.ff_n * 0.001f >= FF_WINDOW_MIN_S) ? 1 : 0;
        }
        rt.phase = PH_SETTLE;  // 通常の設定でブレーキ (SETTLE は基準の設定で、指令 0 へ)
        rt.phase_t = 0.0f;
      }
      // 波形 (指令の加速度を target_vx 列に [0.01 m/s²] で入れる)
      if (++rt.log_divider >= 20) {
        rt.log_divider = 0;
        TcsLog_Record(elapsed_ms, (uint8_t)(ff_result_count + 1), I16(rt.ff_a_cmd * 100.0f),
                      robot->imu.yaw_rate, od);
      }
      break;
    }

    case PH_CRUISE: {
      // v0 まで、滑らない範囲でゆっくり加速し、着いたら (速度が v0 で加速度がほぼ 0 を保ったら) ランプへ
      SetCruiseTune(dir);
      Drive(od, &robot->imu, kDirVec[dir][0] * rt.v0, kDirVec[dir][1] * rt.v0, HeadingHold(robot, 0.0f));
      // 助走の距離は、v0 に最初に着くまで (着いてからの巡航は数えない。距離の予測に使うため)
      rt.cruise_total += fmaxf(speed, 0.0f) * dt;
      if (!rt.cruise_reached) {
        rt.cruise_dist += fmaxf(speed, 0.0f) * dt;
        if (fabsf(speed - rt.v0) < RAMP_CRUISE_TOL) {
          rt.cruise_reached = true;
          if (rt.cur != NULL) rt.cur->cruise_dist_mm = U16(rt.cruise_dist * 1000.0f);
        }
      } else {
        rt.reach_t += dt;
      }
      bool at_speed = fabsf(speed - rt.v0) < RAMP_CRUISE_START_TOL && fabsf(a_imu) < RAMP_CRUISE_A_TOL;
      rt.cruise_ok_t = at_speed ? rt.cruise_ok_t + dt : 0.0f;
      if (rt.cruise_reached && (rt.cruise_ok_t >= RAMP_CRUISE_HOLD_S || rt.reach_t >= RAMP_CRUISE_MAX_DWELL_S)) {
        if (rt.brake_only) {
          BeginBrakeOnly(od, robot, speed);
        } else {
          BeginRamp(od, robot, speed);
        }
      } else if (rt.cruise_total > rt.cruise_limit) {
        // v0 に着かないまま、予測した長さの大半を進んだ。この1本はやめて止まる (範囲を出ないように)
        if (rt.cur != NULL) {
          rt.cur->accel.end_reason = RAMP_END_TIMEOUT;
          rt.cur->cruise_dist_mm = U16(rt.cruise_total * 1000.0f);
        }
        printf("# ramp test: cruise did not reach v0 (dir=%d v0=%d)\n", dir, (int)(rt.v0 * 100.0f));
        rt.phase = PH_SETTLE;
        rt.phase_t = 0.0f;
      }
      break;
    }

    case PH_ACCEL: {
      SetRampTune(rt.det.L);
      if (IsRotation(dir)) {
        float sgn = (dir == RAMP_DIR_ROT_CCW) ? 1.0f : -1.0f;
        Drive(od, &robot->imu, 0.0f, 0.0f, sgn * RAMP_TARGET_ANG);
      } else {
        float target = (rt.v0 > 0.05f) ? rt.v0 + RAMP_TARGET_ABOVE : RAMP_TARGET_SPEED;
        Drive(od, &robot->imu, kDirVec[dir][0] * target, kDirVec[dir][1] * target,
              HeadingHold(robot, 0.0f));
        // 機体の速度は IMU の加速度の積分 (オドメトリは、空転すると実際より速く見える)
        rt.v_imu += a_imu * dt;
        rt.accel_dist += fmaxf(rt.v_imu, 0.0f) * dt;
      }
      DetectStep(dt, a_imu, a_odom, thresh, true);
      AdvanceRamp(dt);

      RampEndReason reason = RAMP_END_NONE;
      float cap = IsRotation(dir) ? RAMP_ANG_SPEED_CAP
                                  : ((rt.v0 > 0.05f) ? rt.v0 + RAMP_SPEED_CAP_ABOVE : RAMP_SPEED_CAP);
      // 滑り始めのあと RAMP_PAST_ONSET_V 上げたか、IMU の加速度がピークから落ちたらやめる。
      // それより先に上限・速度・電圧の余裕に達したら、その理由でやめる (滑り始めは記録されていればよい)
      if (rt.det.onset_found && (rt.det.L_f >= rt.det.onset_L + RAMP_PAST_ONSET_V ||
                                 rt.det.drop_t >= RAMP_PEAK_DROP_HOLD_S)) {
        reason = RAMP_END_ONSET;
      } else if (rt.det.at_max_t >= RAMP_MAX_V_DWELL_S) {
        reason = RAMP_END_MAX_V;
      } else if ((IsRotation(dir) ? speed : rt.v_imu) > cap) {
        reason = RAMP_END_SPEED;
      } else if (rt.det.t > rt.det.delay_s && MinHeadroom(od) < rt.det.L) {
        reason = RAMP_END_HEADROOM;
      } else if (!IsRotation(dir) && rt.det.t > RAMP_ACCEL_MAX_S) {
        reason = RAMP_END_TIMEOUT;
      }
      if (reason != RAMP_END_NONE) {
        if (rt.cur != NULL) {
          SaveDetect(&rt.cur->accel, reason, IsRotation(dir) ? speed : rt.v_imu, acc_scale);
          rt.cur->accel_dist_mm = U16(rt.accel_dist * 1000.0f);
          rt.cur->v_brake_start = U16(fmaxf(IsRotation(dir) ? speed : rt.v_imu, 0.0f) * 1000.0f);
        }
        SeedCommand(od, robot);
        ResetDetect(RAMP_BRAKE_START_V, false, RAMP_BRAKE_DETECT_DELAY_S, RAMP_BRAKE_FAST_UNTIL_V,
                    RAMP_BRAKE_FAST_RATE, RAMP_BRAKE_SLOW_RATE);
        rt.phase = PH_BRAKE;
        rt.phase_t = 0.0f;
      }
      break;
    }

    case PH_BRAKE: {
      SetRampTune(rt.det.L);
      if (IsRotation(dir)) {
        Drive(od, &robot->imu, 0.0f, 0.0f, 0.0f);
      } else {
        Drive(od, &robot->imu, 0.0f, 0.0f, HeadingHold(robot, 0.0f));
        rt.v_imu += a_imu * dt;
        rt.brake_dist += fmaxf(rt.v_imu, 0.0f) * dt;
      }
      // ブレーキは減速の向きを正にする。トルク上限が実際に効いている (出力を縮めた) ときだけ判定する
      float min_speed = IsRotation(dir) ? 1.0f : 0.15f;
      bool valid = od->volt_saturated && speed > min_speed;
      DetectStep(dt, -a_imu, -a_odom, thresh, valid);
      AdvanceRamp(dt);
      // ブレーキのみ: IMU の減速度がピークを過ぎて落ち始めたら、そのトルクで保つ (ロックさせない)
      if (rt.det.brake_only && !rt.det.holding) {
        const bool dropped = rt.det.drop_t >= RAMP_BRAKE_ONLY_DROP_HOLD_S;
        const bool past_onset = rt.det.onset_found && rt.det.L_f >= rt.det.onset_L + RAMP_PAST_ONSET_V;
        const bool beyond_peak = rt.det.peak_acc > RAMP_BRAKE_ONLY_MIN_PEAK &&
                                 rt.det.L_f >= rt.det.peak_L + RAMP_BRAKE_ONLY_BEYOND_PEAK_V;
        if (dropped || past_onset || beyond_peak) {
          rt.det.holding = true;
          rt.det.ramping = false;
          rt.det.L = fmaxf(RAMP_BRAKE_START_V, rt.det.peak_L);
        }
      }

      RampEndReason reason = RAMP_END_NONE;
      float stop_speed = IsRotation(dir) ? 0.3f : 0.05f;
      // 止まったと見なすのは、車輪の速度が止まり、かつ機体の速度 (IMU の積分) も十分に落ちたとき
      // (車輪がロックすると、機体はまだ滑っているのに車輪の速度は 0 になる)
      bool body_stopped = IsRotation(dir) || rt.v_imu < 0.4f;
      if (speed < stop_speed && body_stopped) {
        reason = rt.det.onset_found ? RAMP_END_ONSET : RAMP_END_STOPPED;
      } else if (rt.det.t > RAMP_BRAKE_TIMEOUT_S) {
        reason = RAMP_END_TIMEOUT;
      }
      if (reason != RAMP_END_NONE) {
        if (rt.cur != NULL) {
          SaveDetect(&rt.cur->brake, reason, speed, acc_scale);
          rt.cur->brake_dist_mm = U16(rt.brake_dist * 1000.0f);
        }
        VoltTune_LoadBase(&volt_tune);
        rt.phase = PH_SETTLE;
        rt.phase_t = 0.0f;
      }
      break;
    }

    case PH_SETTLE:
      VoltTune_LoadBase(&volt_tune);
      Drive(od, &robot->imu, 0.0f, 0.0f, IsRotation(dir) ? 0.0f : HeadingHold(robot, 0.0f));
      if (rt.phase_t >= RAMP_SETTLE_S) {
        if (rt.in_speed) {
          rt.sp_pos++;
          if (!PrepareNextSpeedStroke(elapsed_ms)) {
            // 1回目が終わった。機体を 90° 回して繰り返す指定があれば、範囲の真ん中へ移動して回る
            if (rt.session == 1 && (rt.speed_mask & RAMP_SPEED_ROTATE)) {
              rt.tx = 0.5f * (area_x_min + area_x_max);  // 長方形の中心 (原点は、範囲の中心の x で、y は左右の真ん中)
              rt.ty = 0.0f;
              rt.settle_ok_t = 0.0f;
              rt.phase = PH_XFER;
              printf("# ramp test: rotate and repeat (move to x=%d mm)\n", (int)(rt.tx * 1000.0f));
            } else {
              rt.phase = PH_HOME;  // 原点へ戻ってから終わる
              rt.settle_ok_t = 0.0f;
            }
          }
        } else if (rt.member == 0) {
          rt.member = 1;
          rt.phase = PH_START;
        } else {
          rt.phase = PH_RETURN;
          rt.settle_ok_t = 0.0f;
        }
        rt.phase_t = 0.0f;
      }
      break;

    case PH_HOME: {
      // 原点・向き 0 へ低速で戻る。着いたら (または時間切れで) 終わり。戻れなくても、走行の結果には影響しない
      if (GotoStep(od, robot, 0.0f, 0.0f, 0.0f, dt, c, s) || rt.phase_t > RAMP_RETURN_TIMEOUT_S) return Finish(od);
      break;
    }

    case PH_XFER: {
      // 範囲の真ん中へ移動しながら、右へ 90° 向きを変える (機体の左 (+y) が、長方形の長辺の向きになる)
      if (GotoStep(od, robot, rt.tx, rt.ty, -(float)HALF_PI, dt, c, s)) {
        // 新しい座標系: 今の位置を原点、今の向きを 0 とする。範囲は、長方形の縦と横を入れ替えて求める
        const float L = (area_x_max - area_x_min) + 2.0f * RAMP_SESSION_INSET;
        const float W = 2.0f * area_y_abs + 2.0f * RAMP_SESSION_INSET;
        area_x_max = 0.5f * W - RAMP_SESSION_INSET;
        area_x_min = -area_x_max;
        area_y_abs = 0.5f * L - RAMP_SESSION_INSET;
        rt.session = 2;
        rt.pos_x = rt.pos_y = 0.0f;
        rt.heading = 0.0f;
        rt.sp_pos = 0;
        printf("# ramp test: session 2, area x[%d,%d] y+-%d mm\n", (int)(area_x_min * 1000.0f),
               (int)(area_x_max * 1000.0f), (int)(area_y_abs * 1000.0f));
        if (!PrepareNextSpeedStroke(elapsed_ms)) return Finish(od);
      }
      break;
    }

    case PH_RETURN: {
      // 低速の試験: 原点・向き0へ戻る (オドメトリのずれが次の組に積み重ならないように)
      bool arrived = GotoStep(od, robot, 0.0f, 0.0f, 0.0f, dt, c, s);
      if (arrived || rt.phase_t > RAMP_RETURN_TIMEOUT_S) {
        rt.member = 0;
        rt.pair_pos++;
        rt.phase = PH_START;
        rt.phase_t = 0.0f;
        if (rt.pair_pos >= rt.pair_count) {
          rt.set++;
          rt.pair_pos = 0;
          bool more = false;
          if (rt.set == 1) {
            rt.pair_count = RAMP_PAIR_COUNT;
            for (int p = 0; p < RAMP_PAIR_COUNT; p++) rt.pairs[p] = (uint8_t)p;
            more = true;
          } else if (rt.set == 2) {
            more = PlanRetry();
          }
          if (!more) {
            if (rt.set >= 3) MarkUnstable();
            // 低速の試験が終わった。速度別の測定を指定していれば続ける
            if ((rt.speed_mask & ~RAMP_SPEED_V0) != 0U) {
              if (BeginSpeedSection(elapsed_ms)) break;
            }
            return Finish(od);
          }
        }
      }
      break;
    }
  }

  // 波形の記録 (加速・ブレーキの間だけ)。速度別の測定は長いので 20ms 周期
  if (rt.phase == PH_ACCEL || rt.phase == PH_BRAKE) {
    if (++rt.log_divider >= (rt.in_speed ? 20 : 10)) {
      rt.log_divider = 0;
      TcsLog_Record(elapsed_ms, (uint8_t)ramp_result.stroke_count,
                    I16(rt.det.L_f * 1000.0f), robot->imu.yaw_rate, od);
    }
  }
  return RAMP_RUNNING;
}
