#include "optimizer.h"

#include <math.h>
#include <stdio.h>

#include "buzzer.h"
#include "ramp_test.h"
#include "volt_tune.h"

volatile OptResult opt_result;

#define OPT_TOTAL_TIMEOUT_MS 900000U  // 全体の上限 15 分
#define OPT_MAX_RUNS 8                // 走らせる回数の上限 (反復 + 検証)
#define OPT_COOL_MS 10000U            // 走行のあいだの停止 (冷却)
#define OPT_HEAT_WAIT_MS 300000U      // 過熱が消えるまで待つ上限 5 分
#define OPT_MIN_BATTERY_V 21
#define OPT_END_HOLD_MS 6000U  // 終わりの合図 (LED・ブザー) の時間
#define OPT_TOL_ITER 0.03f
#define OPT_TOL_VERIFY 0.05f
#define OPT_KA_MIN 0.3f
#define OPT_KA_MAX 1.2f
#define OPT_STEP_MAX 0.30f  // 1反復の変化の上限 (比率)
#define OPT_ELASTICITY_INIT 0.7f
#define OPT_ELASTICITY_MIN 0.4f
#define OPT_ELASTICITY_MAX 1.0f
#define OPT_AIM 0.9f         // 1回で、差の 90% だけ詰める
#define OPT_RATIO_MIN 0.3f   // これより外の比は、センサの異常として使わない
#define OPT_RATIO_MAX 2.0f

typedef enum { ST_IDLE, ST_COOL, ST_RUN, ST_END } OptState;

typedef struct {
  float ka;
  float prev_ka;
  float prev_r;
  bool has_prev;
} Axis;

static OptState st = ST_IDLE;
static uint32_t flags = 0;
static uint32_t begin_tick = 0;
static uint32_t state_tick = 0;
static uint32_t heat_wait_tick = 0;
static Axis fb, lat;  // 前後 (ka_lin)、左右 (ka_lat)
static bool verifying = false;
static uint32_t seq = 0;

static inline uint16_t X1000(float v) {
  float x = v * 1000.0f + 0.5f;
  return (uint16_t)(x < 0.0f ? 0.0f : (x > 65535.0f ? 65535.0f : x));
}

static void ApplyKa(void) {
  volt_tune.ka_lin = volt_tune_base.ka_lin = fb.ka;
  volt_tune.ka_lat = volt_tune_base.ka_lat = lat.ka;
}

static OptIterLog* CurrentLog(void) {
  uint32_t idx = opt_result.run_count;
  return (OptIterLog*)&opt_result.log[idx < OPT_LOG_MAX ? idx : OPT_LOG_MAX - 1];
}

static void EndWith(OptResultCode code) {
  opt_result.result = (uint32_t)code;
  opt_result.state = 0;
  opt_result.elapsed_ms = HAL_GetTick() - begin_tick;
  opt_result.final_ka_lin_x1000 = X1000(fb.ka);
  opt_result.final_ka_lat_x1000 = X1000(lat.ka);
  st = ST_END;
  state_tick = HAL_GetTick();
  printf("# optimizer: end result=%lu runs=%u ka_lin=%u ka_lat=%u (x1000)\n", (unsigned long)code,
         (unsigned)opt_result.run_count, (unsigned)opt_result.final_ka_lin_x1000,
         (unsigned)opt_result.final_ka_lat_x1000);
  if (code == OPT_RESULT_SAVED || code == OPT_RESULT_PASSED_NOSAVE) {
    Buzzer_Play(BUZZER_SUCCESS);
  } else {
    Buzzer_Play(BUZZER_FAILURE);
  }
}

void Optimizer_Begin(uint32_t task_mask, uint32_t opt_flags) {
  (void)task_mask;  // 今は FF 係数だけ
  flags = opt_flags;
  seq++;
  volatile uint8_t* p = (volatile uint8_t*)&opt_result;
  for (size_t i = 0; i < sizeof(OptResult); i++) p[i] = 0;
  opt_result.seq = seq;
  opt_result.state = 1;
  opt_result.flags = (uint16_t)opt_flags;
  fb = (Axis){volt_tune.ka_lin, 0.0f, 0.0f, false};
  lat = (Axis){volt_tune.ka_lat, 0.0f, 0.0f, false};
  opt_result.start_ka_lin_x1000 = X1000(fb.ka);
  opt_result.start_ka_lat_x1000 = X1000(lat.ka);
  verifying = false;
  begin_tick = HAL_GetTick();
  state_tick = begin_tick;
  heat_wait_tick = begin_tick;
  st = ST_COOL;
  Buzzer_Play(BUZZER_START);
  printf("# optimizer: begin seq=%lu flags=0x%02lx ka_lin=%u ka_lat=%u (x1000)\n", (unsigned long)seq,
         (unsigned long)opt_flags, (unsigned)opt_result.start_ka_lin_x1000,
         (unsigned)opt_result.start_ka_lat_x1000);
}

// 走行の前の確認。OK なら 1、まだなら 0、あきらめるなら -1
static int Precheck(Robot* robot) {
  uint8_t s = robot->omni_drive.wheel_status[0] | robot->omni_drive.wheel_status[1] |
              robot->omni_drive.wheel_status[2] | robot->omni_drive.wheel_status[3];
  bool batt_ok = robot->info.battery_voltage >= OPT_MIN_BATTERY_V;
  bool fault = (s & 0x06U) != 0;  // bit1: 電源電圧範囲外、bit2: 過熱
  if (batt_ok && !fault) return 1;
  if (HAL_GetTick() - heat_wait_tick > OPT_HEAT_WAIT_MS) return -1;
  return 0;
}

// r の平均 (向きが d0〜d1 の有効な本)。use_imu: IMU の加速度 / 指令、でなければ車輪。個数を返す
static int AverageRatio(int d0, int d1, bool use_imu, float* out) {
  float sum = 0.0f;
  int n = 0;
  for (int i = 0; i < ff_result_count && i < FF_RESULT_MAX; i++) {
    const FfStepResult* f = &ff_results[i];
    if (!f->valid || f->dir < d0 || f->dir > d1 || f->a_cmd_x100 < 30) continue;
    float a = use_imu ? f->a_imu_x100 : f->a_odom_x100;
    sum += a / (float)f->a_cmd_x100;
    n++;
  }
  if (n > 0) *out = sum / n;
  return n;
}

static float Clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// 次の ka を求める (r ∝ ka^e)。範囲・変化の上限に収める
static float NextKa(const Axis* a, float r) {
  float e = OPT_ELASTICITY_INIT;
  if (a->has_prev && a->prev_r > 0.0f && fabsf(logf(a->ka / a->prev_ka)) > 0.02f) {
    e = Clampf(logf(r / a->prev_r) / logf(a->ka / a->prev_ka), OPT_ELASTICITY_MIN, OPT_ELASTICITY_MAX);
  }
  float factor = powf(1.0f / r, OPT_AIM / e);
  factor = Clampf(factor, 1.0f - OPT_STEP_MAX, 1.0f + OPT_STEP_MAX);
  return Clampf(a->ka * factor, OPT_KA_MIN, OPT_KA_MAX);
}

static void StartRun(void) {
  ApplyKa();
  RampTest_Reset(RAMP_SPEED_FF_FAST);
  st = ST_RUN;
  printf("# optimizer: run %u%s ka_lin=%u ka_lat=%u\n", (unsigned)(opt_result.run_count + 1),
         verifying ? " (verify)" : "", (unsigned)X1000(fb.ka), (unsigned)X1000(lat.ka));
}

// 走行が最後まで走ったあとの評価
static void Evaluate(void) {
  OptIterLog* lg = CurrentLog();
  float r_fb = 0.0f, r_lat = 0.0f;
  int n_fb, n_lat;
  if (flags & OPT_FLAG_REF_IMU) {
    n_fb = AverageRatio(0, 1, true, &r_fb);
    n_lat = AverageRatio(2, 3, true, &r_lat);
  } else if (flags & OPT_FLAG_REF_WHEEL) {
    n_fb = AverageRatio(0, 1, false, &r_fb);
    n_lat = AverageRatio(2, 3, false, &r_lat);
  } else {  // 相対: 前後は車輪、左右は IMU の左右 / 前後
    float fb_imu = 0.0f;
    n_fb = AverageRatio(0, 1, false, &r_fb);
    int n_fb_imu = AverageRatio(0, 1, true, &fb_imu);
    n_lat = AverageRatio(2, 3, true, &r_lat);
    if (n_fb_imu > 0 && fb_imu > 0.2f) {
      r_lat /= fb_imu;
    } else {
      n_lat = 0;  // 前後の IMU が測れていない: 無効
    }
  }
  lg->kind = verifying ? 1 : 0;
  lg->ramp_result = 1;
  lg->ka_lin_x1000 = X1000(fb.ka);
  lg->ka_lat_x1000 = X1000(lat.ka);
  lg->valid_count = (uint16_t)(n_fb + n_lat);
  lg->t_ms = HAL_GetTick() - begin_tick;
  lg->r_fb_x1000 = (int16_t)(n_fb > 0 ? r_fb * 1000.0f : 0.0f);
  lg->r_lat_x1000 = (int16_t)(n_lat > 0 ? r_lat * 1000.0f : 0.0f);
  lg->next_ka_lin_x1000 = lg->ka_lin_x1000;
  lg->next_ka_lat_x1000 = lg->ka_lat_x1000;
  opt_result.run_count++;
  if (verifying) opt_result.verify_count++;
  printf("# optimizer: run done r_fb=%d r_lat=%d (x1000) valid=%u\n", (int)lg->r_fb_x1000, (int)lg->r_lat_x1000,
         (unsigned)lg->valid_count);

  // 前後・左右とも1本以上測れて、比が範囲内であること (外なら、センサの異常として値を更新しない)
  if (n_fb < 1 || n_lat < 1 || r_fb < OPT_RATIO_MIN || r_fb > OPT_RATIO_MAX || r_lat < OPT_RATIO_MIN ||
      r_lat > OPT_RATIO_MAX) {
    EndWith(OPT_RESULT_BAD_DATA);
    return;
  }
  float tol = verifying ? OPT_TOL_VERIFY : OPT_TOL_ITER;
  bool ok = fabsf(r_fb - 1.0f) <= tol && fabsf(r_lat - 1.0f) <= tol;
  if (ok && verifying) {  // 合格
    if (flags & OPT_FLAG_SAVE) {
      if (VoltTune_SaveTuned(fb.ka, lat.ka)) {
        opt_result.saved = 1;
        EndWith(OPT_RESULT_SAVED);
      } else {
        EndWith(OPT_RESULT_SAVE_FAILED);
      }
    } else {
      EndWith(OPT_RESULT_PASSED_NOSAVE);
    }
    return;
  }
  if (opt_result.run_count >= OPT_MAX_RUNS) {
    EndWith(OPT_RESULT_NO_CONVERGE);
    return;
  }
  if (ok) {  // 収束したので、同じ値でもう1回測る (検証)
    verifying = true;
  } else {
    verifying = false;
    const Axis old_fb = fb, old_lat = lat;
    if (fabsf(r_fb - 1.0f) > OPT_TOL_ITER) {
      fb.ka = NextKa(&old_fb, r_fb);
      fb.prev_ka = old_fb.ka;
      fb.prev_r = r_fb;
      fb.has_prev = true;
    }
    if (fabsf(r_lat - 1.0f) > OPT_TOL_ITER) {
      lat.ka = NextKa(&old_lat, r_lat);
      lat.prev_ka = old_lat.ka;
      lat.prev_r = r_lat;
      lat.has_prev = true;
    }
    lg->next_ka_lin_x1000 = X1000(fb.ka);
    lg->next_ka_lat_x1000 = X1000(lat.ka);
  }
  st = ST_COOL;
  state_tick = HAL_GetTick();
  heat_wait_tick = state_tick;
}

// 終わりの LED0: 成功は短い点滅を3回くり返す、失敗は速い点滅
static bool EndLed(uint32_t t) {
  bool ok = opt_result.result == OPT_RESULT_SAVED || opt_result.result == OPT_RESULT_PASSED_NOSAVE;
  if (!ok) return (t / 100) % 2 == 0;
  uint32_t m = t % 1000;
  return m < 100 || (m >= 250 && m < 350) || (m >= 500 && m < 600);
}

OptStatus Optimizer_Step(LocalController* lc, Robot* robot) {
  uint32_t now = HAL_GetTick();
  Buzzer_Update();
  switch (st) {
    case ST_COOL: {
      LocalController_Stop(lc, robot);
      DigitalOut_Write(&robot->led0, (now / 1000) % 2 == 0);  // ゆっくり点滅
      if (now - begin_tick > OPT_TOTAL_TIMEOUT_MS) {
        EndWith(OPT_RESULT_TIMEOUT);
        return OPT_RUNNING;
      }
      if (now - state_tick < OPT_COOL_MS) return OPT_RUNNING;
      int p = Precheck(robot);
      if (p < 0) {
        EndWith(OPT_RESULT_PRECHECK);
      } else if (p > 0) {
        StartRun();
      }
      return OPT_RUNNING;
    }
    case ST_RUN: {
      if (now - begin_tick > OPT_TOTAL_TIMEOUT_MS) {
        RampTest_Cancel();
        OmniDrive_SetFree(&robot->omni_drive);
        EndWith(OPT_RESULT_TIMEOUT);
        return OPT_RUNNING;
      }
      RampTestStatus rs = RampTest_Step(robot);
      if (rs == RAMP_RUNNING) return OPT_RUNNING;
      OmniDrive_SetFree(&robot->omni_drive);
      if (rs == RAMP_ABORTED) {
        OptIterLog* lg = CurrentLog();
        lg->kind = verifying ? 1 : 0;
        lg->ramp_result = 2;
        lg->ka_lin_x1000 = X1000(fb.ka);
        lg->ka_lat_x1000 = X1000(lat.ka);
        lg->t_ms = now - begin_tick;
        opt_result.run_count++;
        EndWith(OPT_RESULT_ABORTED);
        return OPT_RUNNING;
      }
      Evaluate();
      return OPT_RUNNING;
    }
    case ST_END: {
      LocalController_Stop(lc, robot);
      uint32_t t = now - state_tick;
      DigitalOut_Write(&robot->led0, EndLed(t));
      if (t < OPT_END_HOLD_MS) return OPT_RUNNING;
      DigitalOut_Write(&robot->led0, 0);
      Buzzer_Stop();
      st = ST_IDLE;
      return OPT_DONE;
    }
    case ST_IDLE:
    default:
      return OPT_DONE;
  }
}

void Optimizer_Cancel(void) {
  if (st == ST_IDLE) return;
  if (st == ST_RUN) RampTest_Cancel();
  Buzzer_Stop();
  opt_result.result = OPT_RESULT_CANCELLED;
  opt_result.state = 0;
  opt_result.elapsed_ms = HAL_GetTick() - begin_tick;
  st = ST_IDLE;
}
