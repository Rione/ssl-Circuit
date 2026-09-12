#include "dribbler.h"

DigitalOut BS_LED;
DigitalOut BS_OUT;

// 閾値は「ボール無し時の平均(baseline)」に対する割合で決める。
// 固定値を引く方式だと baseline の水準が変わったときに破綻する。
// 実測: baseline 約560 / ボール有り 約48 なので、50% (=280) は両者の中間に十分収まる。
// 光学系の汚れ等で baseline が下がっても、比例なら余裕の比率が保たれる。
#define PHOTO_THRESHOLD_RATIO_PCT 50
// baseline がこれ未満ならセンサ異常とみなし、検知を無効化する(誤検知より安全)
#define PHOTO_BASELINE_MIN 100
#define BASE_PHOTO_MEASURE_NUM 300
#define PHOTO_LPF_K 0.99

static uint32_t photo_th;
static uint16_t filtered_photo = 0;
static bool photo_lpf_initialized = false;
static LPF photo_lpf;

static uint16_t PhotoLpf_Update(uint16_t photo_val) {
  if (!photo_lpf_initialized) {
    LPF_Init(&photo_lpf, PHOTO_LPF_K, (double)photo_val);
    photo_lpf_initialized = true;
  }

  return (uint16_t)LPF_Update(&photo_lpf, (double)photo_val);
}

void Dribbler_Init() {
  Motor_Init();
  DigitalOut_Init(&BS_LED, BS_LED_GPIO_Port, BS_LED_Pin);
  DigitalOut_Init(&BS_OUT, BS_OUT_GPIO_Port, BS_OUT_Pin);
  DigitalOut_Write(&BS_LED, 1);
  DigitalOut_Write(&BS_OUT, 1);
  photo_th = 0;
  filtered_photo = 0;
  photo_lpf_initialized = false;
}

void Dribbler_Update(uint16_t photo_val, uint16_t current_val) {
  filtered_photo = PhotoLpf_Update(photo_val);

  Motor_Update(current_val);
}

bool Dribbler_SetPhotoThreshold(uint16_t photo_val) {
  static uint16_t count = 0;

  if (count < BASE_PHOTO_MEASURE_NUM) {
    photo_th += photo_val;
    HAL_Delay(1);  // 1ms待機して次のサンプルを取得
  } else {
    photo_th /= BASE_PHOTO_MEASURE_NUM;
    uint32_t baseline = photo_th;

    if (baseline < PHOTO_BASELINE_MIN) {
      // 受光量が異常に低い。閾値0にして検知を止める(常時捕捉より安全)
      printf("  !! photo baseline too low (%lu < %d)\r\n",
             (unsigned long)baseline, PHOTO_BASELINE_MIN);
      printf("     BSセンサ未接続/断線/ADC未動作を疑う。検知は無効化\r\n");
      photo_th = 0;
    } else {
      photo_th = baseline * PHOTO_THRESHOLD_RATIO_PCT / 100U;
    }

    printf("  photo baseline = %lu, threshold = %lu (%d%%)\r\n",
           (unsigned long)baseline, (unsigned long)photo_th,
           PHOTO_THRESHOLD_RATIO_PCT);
    return true;
  }

  // 毎サンプル出すとUARTで詰まるので間引く
  if ((count % 50) == 0) {
    printf("  photo calib %u/%d sum=%lu last=%u\r\n", count,
           BASE_PHOTO_MEASURE_NUM, (unsigned long)photo_th, photo_val);
  }

  count++;
  return false;
}

uint32_t Dribbler_GetPhotoThreshold() { return photo_th; }

uint16_t Dribbler_GetFilteredPhoto() { return filtered_photo; }

bool Dribbler_IsBallCapturedByPhoto() {
  return filtered_photo < photo_th;
}

bool Dribbler_IsBallCapturedByCurrent() {
  return Motor_IsBallCaptured();
}

bool Dribbler_IsBallCaptured() {
  static bool captured_latched = false;
  bool by_photo = Dribbler_IsBallCapturedByPhoto();
  bool by_current = Dribbler_IsBallCapturedByCurrent();

  if (!by_photo) {
    captured_latched = false;
    return false;
  }

  if (by_current) {
    captured_latched = true;
  }

  return captured_latched;
}
