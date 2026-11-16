// 画面描画とタッチ操作 (ステータスバー + 5 ページ + タブバー)
#pragma once

#include <TFT_eSPI.h>

#include "robot_link.h"
#include "touch.h"

class Ui {
 public:
  Ui(TFT_eSPI &tft, Touch &touch, RobotLink &link) : tft_(tft), spr_(&tft), touch_(touch), link_(link) {}

  void begin();
  void update();  // 入力処理 (毎ループ)
  void draw();    // 描画 (FRAME_PERIOD_MS ごと)

  const TestCommand &command() const { return cmd_; }

 private:
  enum Page : uint8_t { kHome, kDribble, kKick, kWheel, kImu, kPageCount };

  // 入力
  void handleStatusBar();
  void handleTabs();
  void handleDribble();
  void handleKick();
  void handleWheel();
  void updateSafety();
  void setPage(Page p);
  void lock();
  bool canArm() const;
  bool tapped(int16_t x, int16_t y, int16_t w, int16_t h) const;
  void click(bool ok = true);

  // 描画
  void drawStatusBar();
  void drawTabs();
  void drawHome();
  void drawDribble();
  void drawKick();
  void drawWheel();
  void drawImu();
  void drawLockHint();

  void card(int16_t x, int16_t y, int16_t w, int16_t h, const char *title);
  void button(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t bg,
              uint16_t fg, uint8_t font = 2);
  void hbar(int16_t x, int16_t y, int16_t w, int16_t h, float ratio, uint16_t color);
  void signedBar(int16_t x, int16_t y, int16_t w, int16_t h, float value, float full_scale,
                 uint16_t color, bool has_marker = false, float marker = 0);
  uint16_t batteryColor(float v) const;

  TFT_eSPI &tft_;
  TFT_eSprite spr_;
  Touch &touch_;
  RobotLink &link_;

  Page page_ = kHome;
  bool armed_ = false;
  TestCommand cmd_;
  uint32_t last_kick_ms_ = 0;
  uint8_t last_kick_ack_ = 0;
  uint32_t kick_flash_ms_ = 0;
};
