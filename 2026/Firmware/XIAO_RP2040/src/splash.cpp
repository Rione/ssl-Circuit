#include "splash.h"

#include "config.h"
#include "logo_bitmap.h"

static constexpr uint16_t kPwmRange = 1023;

void Backlight_Init() {
  analogWriteFreq(20000);  // 可聴域外・ちらつき防止
  analogWriteRange(kPwmRange);
  analogWrite(PIN_BACKLIGHT, 0);
}

void Backlight_Set(uint8_t level) {
  // ガンマ 2.2 近似 (x^2 と x^3 の中間) で暗部の段差を目立たなくする
  float x = level / 255.0f;
  float duty = x * x * (0.6f + 0.4f * x);
  analogWrite(PIN_BACKLIGHT, (int)(duty * kPwmRange + 0.5f));
}

static void DrawLogo(TFT_eSPI &tft) {
  const int16_t x0 = (tft.width() - LOGO_WIDTH) / 2;
  const int16_t y0 = (tft.height() - LOGO_HEIGHT) / 2;

  // 4bit グレー -> RGB565 を 1 行ずつ変換して転送
  uint16_t line[LOGO_WIDTH];
  bool swap = tft.getSwapBytes();
  tft.setSwapBytes(true);  // color565 の値をそのまま配列で送るため
  tft.startWrite();
  for (int16_t y = 0; y < LOGO_HEIGHT; y++) {
    const uint8_t *src = &kLogoBitmap[y * LOGO_WIDTH / 2];
    for (int16_t x = 0; x < LOGO_WIDTH; x++) {
      uint8_t g4 = (x & 1) ? (src[x >> 1] & 0x0F) : (src[x >> 1] >> 4);
      uint8_t g8 = g4 * 17;
      line[x] = tft.color565(g8, g8, g8);
    }
    tft.pushImage(x0, y0 + y, LOGO_WIDTH, 1, line);
  }
  tft.endWrite();
  tft.setSwapBytes(swap);
}

static void Fade(uint32_t duration_ms, bool fade_in, void (*idle)()) {
  uint32_t start = millis();
  for (;;) {
    uint32_t t = millis() - start;
    if (t >= duration_ms) break;
    // smoothstep で始まりと終わりを滑らかに
    float u = (float)t / duration_ms;
    u = u * u * (3.0f - 2.0f * u);
    Backlight_Set((uint8_t)((fade_in ? u : 1.0f - u) * BACKLIGHT_MAX));
    if (idle) idle();
    delay(5);
  }
  Backlight_Set(fade_in ? BACKLIGHT_MAX : 0);
}

void Splash_Show(TFT_eSPI &tft, void (*idle)()) {
  Backlight_Set(0);
  tft.fillScreen(TFT_BLACK);
  DrawLogo(tft);

  Fade(SPLASH_FADE_IN_MS, true, idle);
  uint32_t start = millis();
  while (millis() - start < SPLASH_HOLD_MS) {
    if (idle) idle();
    delay(5);
  }
  Fade(SPLASH_FADE_OUT_MS, false, idle);

  tft.fillScreen(TFT_BLACK);
}
