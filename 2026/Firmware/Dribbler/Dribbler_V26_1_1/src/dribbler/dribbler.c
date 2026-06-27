#include "dribbler.h"

DigitalOut BS_LED;
DigitalOut BS_OUT;

MAF maf_photo;

#define PHOTO_THRESHOLD_MARGIN 300
#define BASE_PHOTO_MEASURE_NUM 100

static uint32_t photo_th;
static uint16_t filtered_photo = 0;

void Dribbler_Init() {
  Motor_Init();
  DigitalOut_Init(&BS_LED, BS_LED_GPIO_Port, BS_LED_Pin);
  DigitalOut_Init(&BS_OUT, BS_OUT_GPIO_Port, BS_OUT_Pin);
  DigitalOut_Write(&BS_LED, 1);
  DigitalOut_Write(&BS_OUT, 1);
  MAF_Init(&maf_photo, 50);
}

void Dribbler_Update(uint16_t photo_val, uint16_t current_val) {
  filtered_photo = MAF_Update(&maf_photo, photo_val);
  Motor_Update(current_val);
}

bool Dribbler_SetPhotoThreshold(uint16_t photo_val) {
  static uint16_t count = 0;

  if (count < BASE_PHOTO_MEASURE_NUM) {
    photo_th += photo_val;
  } else {
    photo_th /= BASE_PHOTO_MEASURE_NUM;
    photo_th -= PHOTO_THRESHOLD_MARGIN;

    return true;
  }

  printf("Photo threshold calibration: %lu / %d\r\n", photo_th, BASE_PHOTO_MEASURE_NUM);

  count++;
  return false;
}

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
