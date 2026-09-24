#include "volt_tune.h"

#include <math.h>
#include <stddef.h>

#include "flash.h"
#include "parammeter.h"

#define VOLT_TUNE_DEFAULTS                                                              \
  {                                                                                     \
    .magic = VOLT_TUNE_MAGIC, .version = VOLT_TUNE_VERSION,                             \
    .traction_limit_v = VOLT_TRACTION_LIMIT_V, .max_accel = VOLT_MODE_MAX_ACCEL,        \
    .max_ang_accel = VOLT_MODE_MAX_ANG_ACCEL, .max_jerk = TCS_MAX_JERK,                 \
    .max_ang_jerk = TCS_MAX_ANG_JERK, .ka_lin = WHEEL_VOLT_KA_LIN_BODY,                 \
    .ka_lat = WHEEL_VOLT_KA_LAT_BODY,                                                   \
    .ka_ang = WHEEL_VOLT_KA_ANG_BODY, .kp_lin = VEL_FB_KP_LIN, .ki_lin = VEL_FB_KI_LIN, \
    .kp_ang = VEL_FB_KP_ANG, .ki_ang = VEL_FB_KI_ANG, .i_max_v = VEL_FB_I_MAX_V,        \
  }

VoltTuneParams volt_tune = VOLT_TUNE_DEFAULTS;
VoltTuneParams volt_tune_base = VOLT_TUNE_DEFAULTS;

void VoltTune_LoadBase(VoltTuneParams* p) {
  *p = volt_tune_base;
}

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
static const Range kKaLat = {0.0f, 1.5f};
static const Range kKaAng = {0.0f, 0.05f};
static const Range kKpLin = {0.0f, 5.0f};
static const Range kKiLin = {0.0f, 20.0f};
static const Range kKpAng = {0.0f, 1.0f};
static const Range kKiAng = {0.0f, 5.0f};
static const Range kIMax = {0.0f, 3.0f};

// フラッシュに保存された調整値 (自動最適化の結果)。有効なら、既定値の ka_lin・ka_lat をこの値にする
static bool saved_valid = false;
static float saved_ka_lin = 0.0f, saved_ka_lat = 0.0f;

void VoltTune_SetDefaults(VoltTuneParams* p) {
  const VoltTuneParams defaults = VOLT_TUNE_DEFAULTS;
  *p = defaults;
  if (saved_valid) {
    p->ka_lin = saved_ka_lin;
    p->ka_lat = saved_ka_lat;
  }
}

// フラッシュの調整値のブロック (0x100 から)。あとで変数を足せるよう、予約を持つ
#define TUNE_FLASH_MAGIC 0x56545346U  // "VTSF"
#define TUNE_FLASH_VERSION 1U
typedef struct {
  uint32_t magic;
  uint32_t version;
  float ka_lin;
  float ka_lat;
  float reserved[12];
  uint32_t checksum;
} TuneFlashBlock;

static uint32_t TuneChecksum(const TuneFlashBlock* b) {
  const uint32_t* words = (const uint32_t*)b;
  uint32_t sum = 0xA5A5A5A5U;
  for (size_t i = 0; i < offsetof(TuneFlashBlock, checksum) / sizeof(uint32_t); i++) {
    sum = ((sum << 3) | (sum >> 29)) ^ words[i];
  }
  return sum;
}

bool VoltTune_HasSaved(void) {
  return saved_valid;
}

bool VoltTune_LoadSaved(void) {
#if AUTOTUNE_LOAD_SAVED
  TuneFlashBlock b;
  Flash_ReadData(FLASH_USER_START_ADDR + VOLT_TUNE_FLASH_OFFSET, &b, sizeof(b));
  saved_valid = false;
  if (b.magic == TUNE_FLASH_MAGIC && b.version == TUNE_FLASH_VERSION && b.checksum == TuneChecksum(&b) &&
      isfinite(b.ka_lin) && isfinite(b.ka_lat) && b.ka_lin >= kKaLin.lo && b.ka_lin <= kKaLin.hi &&
      b.ka_lat >= kKaLat.lo && b.ka_lat <= kKaLat.hi) {
    saved_ka_lin = b.ka_lin;
    saved_ka_lat = b.ka_lat;
    saved_valid = true;
  }
  return saved_valid;
#else
  saved_valid = false;
  return false;
#endif
}

// 512 byte (IMU の較正値 + 調整値) を読んで、調整値のブロックだけ差し替えて、丸ごと書き直す
static bool WriteTuneBlock(const TuneFlashBlock* block) {
  uint8_t image[VOLT_TUNE_FLASH_IMAGE_SIZE];
  Flash_ReadData(FLASH_USER_START_ADDR, image, sizeof(image));
  if (block != NULL) {
    for (size_t i = 0; i < sizeof(TuneFlashBlock); i++) image[VOLT_TUNE_FLASH_OFFSET + i] = ((const uint8_t*)block)[i];
  } else {
    for (size_t i = 0; i < sizeof(TuneFlashBlock); i++) image[VOLT_TUNE_FLASH_OFFSET + i] = 0xFF;  // 消去した状態
  }
  return Flash_WriteData(FLASH_USER_START_ADDR, image, sizeof(image)) == HAL_OK;
}

bool VoltTune_SaveTuned(float ka_lin, float ka_lat) {
  if (!isfinite(ka_lin) || !isfinite(ka_lat)) return false;
  if (ka_lin < kKaLin.lo || ka_lin > kKaLin.hi || ka_lat < kKaLat.lo || ka_lat > kKaLat.hi) return false;
  TuneFlashBlock b;
  for (size_t i = 0; i < sizeof(b); i++) ((uint8_t*)&b)[i] = 0xFF;
  b.magic = TUNE_FLASH_MAGIC;
  b.version = TUNE_FLASH_VERSION;
  b.ka_lin = ka_lin;
  b.ka_lat = ka_lat;
  for (int i = 0; i < 12; i++) b.reserved[i] = 0.0f;
  b.checksum = TuneChecksum(&b);
  if (!WriteTuneBlock(&b)) return false;
  // 書けたか読み直して確かめる
  TuneFlashBlock r;
  Flash_ReadData(FLASH_USER_START_ADDR + VOLT_TUNE_FLASH_OFFSET, &r, sizeof(r));
  if (r.magic != TUNE_FLASH_MAGIC || r.checksum != TuneChecksum(&r) || r.ka_lin != ka_lin || r.ka_lat != ka_lat) {
    return false;
  }
  saved_ka_lin = ka_lin;
  saved_ka_lat = ka_lat;
  saved_valid = true;
  return true;
}

bool VoltTune_ClearSaved(void) {
  if (!WriteTuneBlock(NULL)) return false;
  saved_valid = false;
  return true;
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
  p->ka_lat = Clamp(p->ka_lat, kKaLat, &changed);
  p->ka_ang = Clamp(p->ka_ang, kKaAng, &changed);
  p->kp_lin = Clamp(p->kp_lin, kKpLin, &changed);
  p->ki_lin = Clamp(p->ki_lin, kKiLin, &changed);
  p->kp_ang = Clamp(p->kp_ang, kKpAng, &changed);
  p->ki_ang = Clamp(p->ki_ang, kKiAng, &changed);
  p->i_max_v = Clamp(p->i_max_v, kIMax, &changed);
  return changed;
}
