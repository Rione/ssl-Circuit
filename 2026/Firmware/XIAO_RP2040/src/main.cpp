// RoboCup SSL ロボット UI (XIAO RP2040)
//   - 起動ロゴのフェード表示
//   - MainBoard_V26_2 から受信したテレメトリ (電圧・ボール・ホイール・IMU) の表示
//   - ドリブラー / キッカー / ホイールのテスト指令送信
#include <Arduino.h>
#include <TFT_eSPI.h>

#include "config.h"
#include "robot_link.h"
#include "splash.h"
#include "touch.h"
#include "ui.h"

static TFT_eSPI tft;
static Touch touch(tft);
static RobotLink link;
static Ui ui(tft, touch, link);

static void LinkIdle() { link.update(ui.command()); }

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  Backlight_Init();  // 初期化中の描画が見えないよう最初に消灯

  link.begin();
  tft.init();
  tft.setRotation(SCREEN_ROTATION);
  ui.begin();

  Splash_Show(tft, LinkIdle);

  ui.draw();
  Backlight_Set(BACKLIGHT_MAX);
}

void loop() {
  static uint32_t last_frame_ms = 0;

  ui.update();
  link.update(ui.command());

  uint32_t now = millis();
  if (now - last_frame_ms >= FRAME_PERIOD_MS) {
    last_frame_ms = now;
    ui.draw();
  }
}
