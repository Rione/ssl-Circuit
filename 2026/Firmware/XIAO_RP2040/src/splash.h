// 起動ロゴ (バックライト PWM でフェードイン -> 保持 -> フェードアウト)
#pragma once

#include <TFT_eSPI.h>

void Backlight_Init();
void Backlight_Set(uint8_t level);  // 0-255 (知覚的に線形になるようガンマ補正して出力)

// ロゴを描画してフェード表示する (ブロッキング、合計 約3秒)
// idle は待機中に毎回呼ばれる (通信バッファの消化などに使う)
void Splash_Show(TFT_eSPI &tft, void (*idle)());
