// XPT2046 タッチ入力 (TFT_eSPI 内蔵ドライバの生値を画面座標へ変換し、チャタリングを除去)
#pragma once

#include <TFT_eSPI.h>

class Touch {
 public:
  explicit Touch(TFT_eSPI &tft) : tft_(tft) {}

  void update();

  bool pressed() const { return pressed_; }                  // 押下中
  bool justPressed() const { return pressed_ && !prev_; }    // 押した瞬間
  bool justReleased() const { return !pressed_ && prev_; }   // 離した瞬間
  int16_t x() const { return x_; }
  int16_t y() const { return y_; }
  uint32_t lastActivityMs() const { return last_activity_ms_; }

  bool in(int16_t rx, int16_t ry, int16_t rw, int16_t rh) const {
    return x_ >= rx && x_ < rx + rw && y_ >= ry && y_ < ry + rh;
  }

 private:
  static constexpr uint8_t kPressCount = 2;    // 連続 N 回検出で押下確定
  static constexpr uint8_t kReleaseCount = 3;  // 連続 N 回非検出で解放確定
  static constexpr uint32_t kSamplePeriodMs = 4;

  TFT_eSPI &tft_;
  bool pressed_ = false;
  bool prev_ = false;
  uint8_t count_ = 0;
  int16_t x_ = 0, y_ = 0;
  uint32_t last_activity_ms_ = 0;
  uint32_t last_sample_ms_ = 0;
};
