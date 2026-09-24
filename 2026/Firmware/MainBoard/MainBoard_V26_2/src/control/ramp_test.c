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

// ---- 試験の定数 ----
// トルク上限のランプ (2026-09-24 の初回の結果で見直し、HANDOFF_AUTOTUNE.md 10.8・10.9)。
//  - 始めは 1.8V: どの向きも 2.1V より下では滑らなかったので、その手前は急いで通り過ぎる
//  - 上限は 3.2V (volt_tune の安全範囲の上限と同じ): 採用する値は前後左右 (2.1〜2.5V) で決まる
//  - 速さは 5V/s (1.8→3.2V を約0.28秒): 初回の 10V/s の半分。ゆっくりすぎると速度が出て、今の回転数で
//    転がり続ける電圧が増え、電圧上限 (4.9V) までの余裕が足りなくなる (3.2V に着く頃に約1m/s、
//    斜めでも余裕は約3.3V 残る計算)
#define RAMP_START_V 1.8f
#define RAMP_MAX_V 3.2f
#define RAMP_RATE_V_PER_S 5.0f
// 滑り始めを見つけたあとも、ピーク (IMU の加速度が一番大きい所) を測るために上げ続ける。
// 滑り始めからこれだけ上げたか、IMU の加速度がピークの RAMP_PEAK_DROP_RATIO 倍を
// RAMP_PEAK_DROP_HOLD_S 下回り続けたら、加速をやめる (初回は滑り始めで止めたため、ピークを測れなかった)
#define RAMP_PAST_ONSET_V 0.4f
#define RAMP_PEAK_DROP_RATIO 0.8f
#define RAMP_PEAK_DROP_HOLD_S 0.020f
// 滑り始め: 進む向きの (車輪の加速度 − IMU の加速度) がこれを超えた状態が RAMP_SLIP_HOLD_S 続いた
#define RAMP_SLIP_THRESH_LIN 1.5f   // [m/s^2]
#define RAMP_SLIP_THRESH_ANG 20.0f  // [rad/s^2] (車輪の位置で 1.5m/s² ≒ 20rad/s² × 0.075m)
#define RAMP_SLIP_HOLD_S 0.020f
#define RAMP_DETECT_DELAY_S 0.030f  // ランプを始めてから判定を始めるまで (LPF が落ち着くまで)
#define RAMP_ONSET_HOLD_S 0.030f    // ブレーキ: 滑り始めを見つけてから、トルクを保ってピークを拾う時間
#define RAMP_BRAKE_START_RATIO 0.7f // ブレーキのランプは、加速で見つけた値のこの割合から始める
#define RAMP_AFTER_SLIP_RATIO 0.7f  // 滑り始めを見つけたら、トルク上限をこの割合まで下げる
// 加速の目標 (高くして、FF の要求が常にトルク上限を超える状態にする。上限の値 = 実際のトルク)
#define RAMP_TARGET_SPEED 3.0f      // [m/s]
#define RAMP_TARGET_ANG 25.0f       // [rad/s]
#define RAMP_CMD_ACCEL 8.0f         // S字の加速度上限 [m/s^2]
#define RAMP_CMD_JERK 1000.0f
#define RAMP_CMD_ANG_ACCEL 80.0f
#define RAMP_CMD_ANG_JERK 2000.0f
// 打ち切り
#define RAMP_SPEED_CAP 1.2f         // [m/s]
#define RAMP_ANG_SPEED_CAP 12.0f    // [rad/s]
#define RAMP_MAX_V_DWELL_S 0.05f    // 上限に達してから打ち切るまで (ピークを拾う)
#define RAMP_BRAKE_TIMEOUT_S 1.5f
#define RAMP_SETTLE_S 0.3f
#define RAMP_RETURN_TIMEOUT_S 4.0f
#define RAMP_PHASE_TIMEOUT_S 3.0f
// 3回目を測るかの判定: 2回の滑り始めの差がこれ以上 (どちらか)。初回の 0.2V/10% は厳しすぎて、
// 10向き中9向きで3回目に入った
#define RAMP_RETRY_DIFF_V 0.30f
#define RAMP_RETRY_DIFF_RATIO 0.15f
// 安全停止の範囲 [m] (後ろの空きは約0.5m)
#define RAMP_AREA_X_MIN (-0.4f)
#define RAMP_AREA_X_MAX 1.6f
#define RAMP_AREA_Y_ABS 1.6f
#define RAMP_HEADING_ABORT 0.8f  // 並進の最中の向きのずれ [rad]

#define RAMP_PAIR_COUNT 5
#define RAMP_SET_MAX 3

static const float kDirVec[8][2] = {
    {1.0f, 0.0f},         {-1.0f, 0.0f},       {0.0f, 1.0f},          {0.0f, -1.0f},
    {0.7071f, 0.7071f},   {-0.7071f, -0.7071f}, {0.7071f, -0.7071f},  {-0.7071f, 0.7071f},
};

typedef enum {
  PH_START = 0,  // 次の1本の準備
  PH_ACCEL,
  PH_BRAKE,
  PH_SETTLE,
  PH_RETURN,     // 組の終わりに原点・向き0へ戻る
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
} Detect;

static struct {
  bool started;
  uint32_t start_tick;
  Timer dt_timer;
  int set;                     // 0, 1, 2
  uint8_t pairs[RAMP_PAIR_COUNT];
  int pair_count;
  int pair_pos;
  int member;                  // 組の1本目 (0) か2本目 (1)
  Phase phase;
  float phase_t;
  Detect det;
  float settle_ok_t;
  // 旋回の角加速度 (ジャイロと車輪の角速度の微分に LPF)
  float prev_gyro, prev_odom_w, alpha_gyro_f, alpha_odom_f;
  // 床の座標での位置と向き (安全停止と原点への戻りに使う)
  float pos_x, pos_y, heading;
  uint8_t log_divider;
  RampStrokeResult* cur;
} rt;

void RampTest_Reset(void) {
  rt.started = false;
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

static int CurrentDir(void) { return rt.pairs[rt.pair_pos] * 2 + rt.member; }
static bool IsRotation(int dir) { return dir >= RAMP_DIR_ROT_CCW; }

static void ResetDetect(float L_start, bool past_onset) {
  Detect zero = {0};
  rt.det = zero;
  rt.det.L = L_start;
  rt.det.L_f = L_start;
  rt.det.ramping = true;
  rt.det.past_onset = past_onset;
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
  if (d->onset_found) {
    // 見つけた後もピークを拾う (加速は上げ続けながら、ブレーキは保つ間だけ)。
    // 保つ時間は valid に関係なく進める (止まらずにトルクを下げられるように)
    d->after_onset_t += dt;
    bool track = d->past_onset || d->after_onset_t <= RAMP_ONSET_HOLD_S;
    if (valid && track && a_imu > d->peak_acc) {
      d->peak_acc = a_imu;
      d->peak_L = d->L_f;
    }
    if (d->past_onset && valid) {
      d->drop_t = (a_imu < RAMP_PEAK_DROP_RATIO * d->peak_acc) ? d->drop_t + dt : 0.0f;
    }
    return;
  }
  if (!valid || d->t < RAMP_DETECT_DELAY_S) {
    d->slip_t = 0.0f;
    return;
  }
  if (a_imu > d->peak_acc) {
    d->peak_acc = a_imu;
    d->peak_L = d->L_f;
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
      // 加速は、滑り始めのあともランプを続ける (ピークを測るため)。ブレーキは止めて、保つ・下げるに移る
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
  d->L += RAMP_RATE_V_PER_S * dt;
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
  r->speed = I16(speed * 1000.0f);
  r->end_reason = (uint16_t)reason;
}

// volt_tune をランプ用にする (トルク上限だけランプの値、S字は十分速く)
static void SetRampTune(float L) {
  VoltTune_SetDefaults(&volt_tune);
  volt_tune.traction_limit_v = L;  // Sanitize は通さない (試験の中だけ 3.2V を超えてよい)
  volt_tune.max_accel = RAMP_CMD_ACCEL;
  volt_tune.max_jerk = RAMP_CMD_JERK;
  volt_tune.max_ang_accel = RAMP_CMD_ANG_ACCEL;
  volt_tune.max_ang_jerk = RAMP_CMD_ANG_JERK;
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

// 向き dir の set 回目の加速の滑り始め [V] (見つからなかったとき・記録が無いときは −1)
static float OnsetVOf(int dir, int set) {
  for (int i = 0; i < ramp_result.stroke_count; i++) {
    const RampStrokeResult* s = &ramp_result.stroke[i];
    if (s->dir == dir && s->set == set + 1) {
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

static void BeginStroke(OmniDrive* od, const Robot* robot, uint32_t elapsed_ms) {
  int dir = CurrentDir();
  rt.cur = NULL;
  if (ramp_result.stroke_count < RAMP_MAX_STROKES) {
    rt.cur = &ramp_result.stroke[ramp_result.stroke_count++];
    RampStrokeResult zero = {0};
    *rt.cur = zero;
    rt.cur->dir = (uint8_t)dir;
    rt.cur->set = (uint8_t)(rt.set + 1);
    rt.cur->batt = 255;
    rt.cur->t_start_ms = (uint16_t)(elapsed_ms > 65535U ? 65535U : elapsed_ms);
  }
  for (int k = 0; k < 3; k++) od->vel_fb_integral[k] = 0.0f;
  SeedCommand(od, robot);
  ResetDetect(RAMP_START_V, true);
  float odom_vx, odom_vy, odom_w;
  OmniDrive_GetVelF(od, &odom_vx, &odom_vy, &odom_w);
  rt.prev_gyro = robot->imu.yaw_rate;
  rt.prev_odom_w = odom_w;
  rt.alpha_gyro_f = 0.0f;
  rt.alpha_odom_f = 0.0f;
  rt.phase = PH_ACCEL;
  rt.phase_t = 0.0f;
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
    rt.start_tick = HAL_GetTick();
    Timer_Init(&rt.dt_timer);
    Timer_Reset(&rt.dt_timer);
    uint32_t seq = ramp_result.seq + 1;
    RampRunResult zero = {0};
    ramp_result = zero;
    ramp_result.seq = seq;
    rt.set = 0;
    rt.pair_count = RAMP_PAIR_COUNT;
    for (int p = 0; p < RAMP_PAIR_COUNT; p++) rt.pairs[p] = (uint8_t)p;
    rt.pair_pos = 0;
    rt.member = 0;
    rt.pos_x = rt.pos_y = rt.heading = 0.0f;
    rt.phase = PH_START;
    rt.phase_t = 0.0f;
    rt.log_divider = 0;
    TCS_Reset(&od->tcs);
    TcsLog_Reset();
    printf("# ramp test started (seq=%lu)\n", (unsigned long)seq);
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
  if (rt.pos_x < RAMP_AREA_X_MIN || rt.pos_x > RAMP_AREA_X_MAX || fabsf(rt.pos_y) > RAMP_AREA_Y_ABS) {
    return Abort(od, 1);
  }
  int dir = CurrentDir();
  bool translating = (rt.phase == PH_ACCEL || rt.phase == PH_BRAKE) && !IsRotation(dir);
  if (translating && fabsf(rt.heading) > RAMP_HEADING_ABORT) return Abort(od, 2);
  float timeout = (rt.phase == PH_RETURN) ? RAMP_RETURN_TIMEOUT_S : RAMP_PHASE_TIMEOUT_S;
  if (rt.phase != PH_RETURN && rt.phase_t > timeout) return Abort(od, 3);
  if (st & 0x06U) return Abort(od, 4);  // bit1: 電源電圧範囲外、bit2: 過熱

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
      BeginStroke(od, robot, elapsed_ms);
      return RAMP_RUNNING;

    case PH_ACCEL: {
      SetRampTune(rt.det.L);
      if (IsRotation(dir)) {
        float sgn = (dir == RAMP_DIR_ROT_CCW) ? 1.0f : -1.0f;
        Drive(od, &robot->imu, 0.0f, 0.0f, sgn * RAMP_TARGET_ANG);
      } else {
        Drive(od, &robot->imu, kDirVec[dir][0] * RAMP_TARGET_SPEED,
              kDirVec[dir][1] * RAMP_TARGET_SPEED, HeadingHold(robot, 0.0f));
      }
      DetectStep(dt, a_imu, a_odom, thresh, true);
      AdvanceRamp(dt);

      RampEndReason reason = RAMP_END_NONE;
      float cap = IsRotation(dir) ? RAMP_ANG_SPEED_CAP : RAMP_SPEED_CAP;
      // 滑り始めのあと RAMP_PAST_ONSET_V 上げたか、IMU の加速度がピークから落ちたらやめる。
      // それより先に上限・速度・電圧の余裕に達したら、その理由でやめる (滑り始めは記録されていればよい)
      if (rt.det.onset_found && (rt.det.L_f >= rt.det.onset_L + RAMP_PAST_ONSET_V ||
                                 rt.det.drop_t >= RAMP_PEAK_DROP_HOLD_S)) {
        reason = RAMP_END_ONSET;
      } else if (rt.det.at_max_t >= RAMP_MAX_V_DWELL_S) {
        reason = RAMP_END_MAX_V;
      } else if (speed > cap) {
        reason = RAMP_END_SPEED;
      } else if (rt.det.t > RAMP_DETECT_DELAY_S && MinHeadroom(od) < rt.det.L) {
        reason = RAMP_END_HEADROOM;
      }
      if (reason != RAMP_END_NONE) {
        if (rt.cur != NULL) SaveDetect(&rt.cur->accel, reason, speed, acc_scale);
        float found = rt.det.onset_found ? rt.det.onset_L : rt.det.peak_L;
        float L0 = fmaxf(1.0f, RAMP_BRAKE_START_RATIO * found);
        SeedCommand(od, robot);
        ResetDetect(L0, false);
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
      }
      // ブレーキは減速の向きを正にする。トルク上限が実際に効いている (出力を縮めた) ときだけ判定する
      float min_speed = IsRotation(dir) ? 1.0f : 0.15f;
      bool valid = od->volt_saturated && speed > min_speed;
      DetectStep(dt, -a_imu, -a_odom, thresh, valid);
      AdvanceRamp(dt);

      RampEndReason reason = RAMP_END_NONE;
      float stop_speed = IsRotation(dir) ? 0.3f : 0.05f;
      if (speed < stop_speed) {
        reason = rt.det.onset_found ? RAMP_END_ONSET : RAMP_END_STOPPED;
      } else if (rt.det.t > RAMP_BRAKE_TIMEOUT_S) {
        reason = RAMP_END_TIMEOUT;
      }
      if (reason != RAMP_END_NONE) {
        if (rt.cur != NULL) SaveDetect(&rt.cur->brake, reason, speed, acc_scale);
        VoltTune_SetDefaults(&volt_tune);
        rt.phase = PH_SETTLE;
        rt.phase_t = 0.0f;
      }
      break;
    }

    case PH_SETTLE:
      VoltTune_SetDefaults(&volt_tune);
      Drive(od, &robot->imu, 0.0f, 0.0f, IsRotation(dir) ? 0.0f : HeadingHold(robot, 0.0f));
      if (rt.phase_t >= RAMP_SETTLE_S) {
        if (rt.member == 0) {
          rt.member = 1;
          rt.phase = PH_START;
        } else {
          rt.phase = PH_RETURN;
          rt.settle_ok_t = 0.0f;
        }
        rt.phase_t = 0.0f;
      }
      break;

    case PH_RETURN: {
      // 低速で原点・向き0へ戻る (オドメトリのずれが次の組に積み重ならないように)
      VoltTune_SetDefaults(&volt_tune);
      float ex = -rt.pos_x, ey = -rt.pos_y;
      float dist = sqrtf(ex * ex + ey * ey);
      float sp = fminf(0.8f, fminf(sqrtf(2.0f * 3.0f * dist), 4.0f * dist));
      float vxw = (dist > 1e-3f) ? ex / dist * sp : 0.0f;
      float vyw = (dist > 1e-3f) ? ey / dist * sp : 0.0f;
      Drive(od, &robot->imu, vxw * c + vyw * s, -vxw * s + vyw * c, HeadingHold(robot, 0.0f));
      if (dist < 0.03f && fabsf(rt.heading) < 0.05f) {
        rt.settle_ok_t += dt;
      } else {
        rt.settle_ok_t = 0.0f;
      }
      if (rt.settle_ok_t >= 0.15f || rt.phase_t > RAMP_RETURN_TIMEOUT_S) {
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
            OmniDrive_SetFree(od);
            VoltTune_SetDefaults(&volt_tune);
            ramp_result.result = 1;
            printf("# ramp test finished: strokes=%u retry=0x%03x unstable=0x%03x\n",
                   ramp_result.stroke_count, ramp_result.retry_mask, ramp_result.unstable_mask);
            return RAMP_FINISHED;
          }
        }
      }
      break;
    }
  }

  // 波形の記録 (加速・ブレーキの間だけ、10ms 周期)
  if (rt.phase == PH_ACCEL || rt.phase == PH_BRAKE) {
    if (++rt.log_divider >= 10) {
      rt.log_divider = 0;
      TcsLog_Record(elapsed_ms, (uint8_t)ramp_result.stroke_count,
                    I16(rt.det.L_f * 1000.0f), robot->imu.yaw_rate, od);
    }
  }
  return RAMP_RUNNING;
}
