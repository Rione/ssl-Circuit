#include "volt_tune.h"

#include <math.h>
#include <stddef.h>

#include "parammeter.h"

#define VOLT_TUNE_DEFAULTS                                                              \
  {                                                                                     \
    .magic = VOLT_TUNE_MAGIC, .version = VOLT_TUNE_VERSION,                             \
    .traction_limit_v = VOLT_TRACTION_LIMIT_V, .max_accel = VOLT_MODE_MAX_ACCEL,        \
    .max_ang_accel = VOLT_MODE_MAX_ANG_ACCEL, .max_jerk = TCS_MAX_JERK,                 \
    .max_ang_jerk = TCS_MAX_ANG_JERK, .ka_lin = WHEEL_VOLT_KA_LIN_BODY,                 \
    .ka_ang = WHEEL_VOLT_KA_ANG_BODY, .kp_lin = VEL_FB_KP_LIN, .ki_lin = VEL_FB_KI_LIN, \
    .kp_ang = VEL_FB_KP_ANG, .ki_ang = VEL_FB_KI_ANG, .i_max_v = VEL_FB_I_MAX_V,        \
  }

VoltTuneParams volt_tune = VOLT_TUNE_DEFAULTS;

// 安全範囲 [下限, 上限]。自動チューニングで強くしていっても、この外には出さない。
// トルク上限は手動調整で「滑りすぎ」とされた 3.2V まで (HANDOFF_AUTOTUNE.md 7章の8)
typedef struct {
  float lo, hi;
} Range;

static const Range kTractionLimit = {0.5f, 3.2f};
static const Range kMaxAccel = {0.5f, 8.0f};
static const Range kMaxAngAccel = {5.0f, 50.0f};
static const Range kMaxJerk = {50.0f, 1000.0f};
static const Range kMaxAngJerk = {100.0f, 2000.0f};
static const Range kKaLin = {0.0f, 1.5f};
static const Range kKaAng = {0.0f, 0.05f};
static const Range kKpLin = {0.0f, 5.0f};
static const Range kKiLin = {0.0f, 20.0f};
static const Range kKpAng = {0.0f, 1.0f};
static const Range kKiAng = {0.0f, 5.0f};
static const Range kIMax = {0.0f, 3.0f};

void VoltTune_SetDefaults(VoltTuneParams* p) {
  const VoltTuneParams defaults = VOLT_TUNE_DEFAULTS;
  *p = defaults;
}

// 範囲に収める。直したら *changed を 1 にする
static float Clamp(float v, Range r, int* changed) {
  if (v < r.lo) {
    *changed = 1;
    return r.lo;
  }
  if (v > r.hi) {
    *changed = 1;
    return r.hi;
  }
  return v;
}

int VoltTune_Sanitize(VoltTuneParams* p) {
  if (p->magic != VOLT_TUNE_MAGIC || p->version != VOLT_TUNE_VERSION) {
    VoltTune_SetDefaults(p);
    return 1;
  }
  const float* values = &p->traction_limit_v;
  const int n = (int)((sizeof(VoltTuneParams) - offsetof(VoltTuneParams, traction_limit_v)) /
                      sizeof(float));
  for (int i = 0; i < n; i++) {
    if (!isfinite(values[i])) {
      VoltTune_SetDefaults(p);
      return 1;
    }
  }

  int changed = 0;
  p->traction_limit_v = Clamp(p->traction_limit_v, kTractionLimit, &changed);
  p->max_accel = Clamp(p->max_accel, kMaxAccel, &changed);
  p->max_ang_accel = Clamp(p->max_ang_accel, kMaxAngAccel, &changed);
  p->max_jerk = Clamp(p->max_jerk, kMaxJerk, &changed);
  p->max_ang_jerk = Clamp(p->max_ang_jerk, kMaxAngJerk, &changed);
  p->ka_lin = Clamp(p->ka_lin, kKaLin, &changed);
  p->ka_ang = Clamp(p->ka_ang, kKaAng, &changed);
  p->kp_lin = Clamp(p->kp_lin, kKpLin, &changed);
  p->ki_lin = Clamp(p->ki_lin, kKiLin, &changed);
  p->kp_ang = Clamp(p->kp_ang, kKpAng, &changed);
  p->ki_ang = Clamp(p->ki_ang, kKiAng, &changed);
  p->i_max_v = Clamp(p->i_max_v, kIMax, &changed);
  return changed;
}
