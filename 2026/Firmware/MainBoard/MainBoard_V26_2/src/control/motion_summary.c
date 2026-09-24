#include "motion_summary.h"

#include <math.h>
#include <stdbool.h>

#include "volt_tune.h"

MotionRunResult motion_results[MOTION_RESULT_RUNS];
volatile uint32_t motion_result_count = 0;

// 加速中と数える最小の指令加速度 [m/s^2] (TCS_ACCEL_DETECT_MIN と同じ)
#define MOTION_ACC_MIN 1.0f
// 「動いている」と数える最小のオドメトリ速度 [m/s]
#define MOTION_MOVING_MIN 0.3f
// 推定対地速度との差がこれを超えたら「差が大きい」と数える [m/s]
#define MOTION_GAP_BIG 0.3f

// 区間の計算用 (1kHz で足し込む)
typedef struct {
  int seg;
  float start_x, start_y, target_x, target_y, target_heading, start_heading;
  float dir_x, dir_y, length;  // 始めから目標への向きの単位ベクトルと距離 (並進の区間)
  bool is_rotation;            // その場旋回 (移動が小さく、向きの変化が大きい)
  float t;                     // 経過時間 [s]
  float slip_t, lim_t;         // is_slipping・出力を縮めた時間 [s]
  float vmax;                  // [m/s]
  float acc_sum, acc_t;        // 加速中の超過加速度の積分・時間
  float gap_sum, gap_t, gap_big_t;
  float cross_max, pos_over, head_over;
  uint8_t status_or;
  float batt_min;
} Accum;

static Accum acc;
static MotionRunResult* cur = NULL;

static int16_t I16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static uint16_t U16(float v) {
  if (v < 0.0f) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

void MotionSummary_BeginRun(void) {
  cur = &motion_results[motion_result_count % MOTION_RESULT_RUNS];
  MotionRunResult zero = {0};
  *cur = zero;
  cur->seq = motion_result_count + 1;
  cur->traction_x100 = U16(volt_tune.traction_limit_v * 100.0f);
  cur->max_accel_x100 = U16(volt_tune.max_accel * 100.0f);
  cur->max_ang_accel_x100 = U16(volt_tune.max_ang_accel * 100.0f);
  motion_result_count++;
}

void MotionSummary_BeginSegment(int seg, float start_x, float start_y, float target_x,
                                float target_y, float target_heading, float start_heading) {
  Accum zero = {0};
  acc = zero;
  acc.seg = seg;
  acc.start_x = start_x;
  acc.start_y = start_y;
  acc.target_x = target_x;
  acc.target_y = target_y;
  acc.target_heading = target_heading;
  acc.start_heading = start_heading;
  float dx = target_x - start_x, dy = target_y - start_y;
  acc.length = sqrtf(dx * dx + dy * dy);
  if (acc.length > 1e-3f) {
    acc.dir_x = dx / acc.length;
    acc.dir_y = dy / acc.length;
  }
  acc.is_rotation = (acc.length < 0.05f) && (fabsf(target_heading - start_heading) > 0.5f);
  acc.batt_min = 99.0f;
}

void MotionSummary_Step(float dt, const Robot* robot, float pos_x, float pos_y, float heading) {
  if (cur == NULL) return;
  const OmniDrive* od = &robot->omni_drive;
  const TractionControl* tcs = &od->tcs;
  acc.t += dt;

  float vo = sqrtf(tcs->odom_vx * tcs->odom_vx + tcs->odom_vy * tcs->odom_vy);
  float vg = sqrtf(tcs->ground_vx * tcs->ground_vx + tcs->ground_vy * tcs->ground_vy);
  if (vo > acc.vmax) acc.vmax = vo;

  if (tcs->is_slipping) acc.slip_t += dt;
  if (od->volt_traction_limited) acc.lim_t += dt;

  // A: 加速中 (指令の加速度が大きいとき) の、オドメトリの加速度が IMU の加速度を超えた分。
  //    車輪が空転すると、オドメトリの加速度だけが大きくなる。TCS の判定 (is_slipping) を使わない
  float cmd_a = sqrtf(tcs->current_ax * tcs->current_ax + tcs->current_ay * tcs->current_ay);
  float cmd_alpha = fabsf(tcs->current_alpha);
  if (cmd_a > MOTION_ACC_MIN && cmd_alpha < 5.0f) {
    float ao = sqrtf(tcs->a_odom_x * tcs->a_odom_x + tcs->a_odom_y * tcs->a_odom_y);
    float ai = sqrtf(tcs->a_imu_x * tcs->a_imu_x + tcs->a_imu_y * tcs->a_imu_y);
    acc.acc_sum += fmaxf(0.0f, ao - ai) * dt;
    acc.acc_t += dt;
  }
  // B: オドメトリ速度と推定対地速度の差
  if (vo > MOTION_MOVING_MIN) {
    float gap = fmaxf(0.0f, vo - vg);
    acc.gap_sum += gap * dt;
    acc.gap_t += dt;
    if (gap > MOTION_GAP_BIG) acc.gap_big_t += dt;
  }

  // 正確さ
  float ex = pos_x - acc.start_x, ey = pos_y - acc.start_y;
  if (acc.length > 0.05f) {
    float along = ex * acc.dir_x + ey * acc.dir_y;
    float cross = fabsf(-ex * acc.dir_y + ey * acc.dir_x);
    if (cross > acc.cross_max) acc.cross_max = cross;
    float over = along - acc.length;
    if (over > acc.pos_over) acc.pos_over = over;
  }
  if (acc.is_rotation) {
    // 目標を越えた量 (回る向きに沿って測る)
    float sign = (acc.target_heading >= acc.start_heading) ? 1.0f : -1.0f;
    float over = (heading - acc.target_heading) * sign;
    if (over > acc.head_over) acc.head_over = over;
  } else {
    float herr = fabsf(heading - acc.target_heading);
    if (herr > acc.head_over) acc.head_over = herr;
  }

  acc.status_or |= (uint8_t)(od->wheel_status[0] | od->wheel_status[1] | od->wheel_status[2] |
                             od->wheel_status[3]);
  float batt = (float)robot->info.battery_voltage;
  if (batt > 1.0f && batt < acc.batt_min) acc.batt_min = batt;
}

void MotionSummary_EndSegment(float pos_x, float pos_y, float heading, uint32_t dur_ms) {
  if (cur == NULL || acc.seg < 0 || acc.seg >= MOTION_SEG_MAX) return;
  MotionSegSummary* s = &cur->seg[acc.seg];
  float t = fmaxf(acc.t, 1e-3f);
  s->dur_ms = (uint16_t)(dur_ms > 65535U ? 65535U : dur_ms);
  s->slip_pct_x10 = U16(1000.0f * acc.slip_t / t);
  s->lim_pct_x10 = U16(1000.0f * acc.lim_t / t);
  s->vmax_mmps = I16(acc.vmax * 1000.0f);
  s->s_acc_x100 = (acc.acc_t > 1e-3f) ? I16(100.0f * acc.acc_sum / acc.acc_t) : 0;
  s->gap_mean_x1000 = (acc.gap_t > 1e-3f) ? I16(1000.0f * acc.gap_sum / acc.gap_t) : 0;
  s->gap_frac_x1000 = (acc.gap_t > 1e-3f) ? U16(1000.0f * acc.gap_big_t / acc.gap_t) : 0;
  s->acc_ms = U16(acc.acc_t * 1000.0f);
  s->cross_max_mm = I16(acc.cross_max * 1000.0f);
  s->pos_over_mm = I16(acc.pos_over * 1000.0f);
  s->head_over_mrad = I16(acc.head_over * 1000.0f);
  float ex = acc.target_x - pos_x, ey = acc.target_y - pos_y;
  s->end_dist_mm = I16(sqrtf(ex * ex + ey * ey) * 1000.0f);
  s->end_head_mrad = I16((acc.target_heading - heading) * 1000.0f);
  s->status_or = acc.status_or;
  s->batt_min_x10 = (acc.batt_min < 90.0f) ? U16(acc.batt_min * 10.0f) : 0;
  if ((uint16_t)(acc.seg + 1) > cur->seg_count) cur->seg_count = (uint16_t)(acc.seg + 1);
}

void MotionSummary_EndRun(uint32_t result, uint32_t total_ms) {
  if (cur == NULL) return;
  cur->result = result;
  cur->total_ms = total_ms;
  cur = NULL;
}
