#include "dribbler.h"

DigitalOut BS_LED;

MAF maf_photo;

#define PHOTO_THRESHOLD_MARGIN 200
#define BASE_PHOTO_MEASURE_NUM 500

static uint32_t photo_th;
static uint16_t filtered_photo = 0;

void Dribbler_Init() {
  DigitalOut_Init(&BS_LED, BS_LED_GPIO_Port, BS_LED_Pin);
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
  bool by_photo = Dribbler_IsBallCapturedByPhoto();
  bool by_current = Dribbler_IsBallCapturedByCurrent();

  return by_photo && by_current;
}
