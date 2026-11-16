#include "touch.h"

#include "config.h"

void Touch::update() {
  prev_ = pressed_;

  // 一定周期でサンプリングしないとチャタリング除去の回数判定が意味を持たない
  uint32_t now = millis();
  if (now - last_sample_ms_ < kSamplePeriodMs) return;
  last_sample_ms_ = now;

  bool raw_pressed = tft_.getTouchRawZ() > TOUCH_Z_THRESHOLD;
  uint16_t rx = 0, ry = 0;
  if (raw_pressed) tft_.getTouchRaw(&rx, &ry);

  // 状態と逆の検出が連続したときだけ状態を反転する
  if (raw_pressed != pressed_) {
    if (++count_ >= (pressed_ ? kReleaseCount : kPressCount)) {
      pressed_ = raw_pressed;
      count_ = 0;
    }
  } else {
    count_ = 0;
  }

  if (raw_pressed) {
    int32_t sx = TOUCH_SWAP_XY ? ry : rx;
    int32_t sy = TOUCH_SWAP_XY ? rx : ry;
    if (TOUCH_INVERT_X) sx = 4095 - sx;
    if (TOUCH_INVERT_Y) sy = 4095 - sy;
    x_ = constrain(map(sx, TOUCH_X_MIN, TOUCH_X_MAX, 0, tft_.width()), 0, tft_.width() - 1);
    y_ = constrain(map(sy, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, tft_.height()), 0, tft_.height() - 1);
    last_activity_ms_ = millis();
#if TOUCH_DEBUG
    Serial.printf("touch raw x=%u y=%u -> %d,%d\n", rx, ry, x_, y_);
#endif
  }
}
