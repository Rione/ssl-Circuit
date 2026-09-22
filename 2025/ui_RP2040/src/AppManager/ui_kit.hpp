#ifndef _UI_KIT_
#define _UI_KIT_

#include <Arduino.h>

#include "UI/components/button.hpp"

// #include "UI/font/bold40.h"
// #include "UI/font/bold20.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"

#include "Media/MediaExecutor.hpp"

typedef struct {
  // 受け取るデータ
  float batteryVoltage;
  union {
    struct {
      uint8_t chargeState : 1;  // stmから送られてくる充電状態
      uint8_t chargeVal : 7; // capChargeCertitude
    };
    uint8_t data;
  } capaData;
  uint8_t buzzerState;

  // 新規追加: ボールセンサとモーター情報
  bool ballSensor;
  struct {
    float velocity;
    bool statusOk;
  } motor[4];

  // modeの状態
  union {
    struct {
      uint8_t mode : 5;
      bool emergencyStop : 1;
      bool charge_state : 1; // 1.切替、0.切替なし
      bool kick : 1;         // キック
    };
    uint8_t data;
  } modeStatus;
  uint8_t testCommand = 0; // 新規追加: テストコマンド
  uint8_t modePrev = 0;

} RobotInfo_t;

class UiKit {
public:
  RobotInfo_t info = {0};
  RobotInfo_t infoPrev = {0};

  TFT_eSPI tft = TFT_eSPI();
  TFT_eSprite sprite = TFT_eSprite(&tft);
  DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

  XPT2046_Touchscreen ts = XPT2046_Touchscreen(TOUCH_CS);
  TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

  BUTTON main_ChargeButton = BUTTON(&display, &touch, 30, 155, 85, 60,
                                    main_chgRedImg, main_chgWhiteImg);
  BUTTON main_KickButton = BUTTON(&display, &touch, 125, 155, 85, 60,
                                  main_kickRedImg, main_kickWhiteImg);

  const uint16_t chargeBack = sprite.color565(255, 210, 51); // 充電中の背景色
  const uint16_t tabBack = sprite.color565(195, 216, 242);   // 旧タブの背景色

  // Dashboard (Light Theme) Colors
  const uint16_t colBg = sprite.color565(255, 255, 255); // Background
  const uint16_t colFg = sprite.color565(20, 20, 20);    // Foreground (text)
  const uint16_t colSidebar =
      sprite.color565(248, 249, 250); // Sidebar background
  const uint16_t colBorder = sprite.color565(230, 230, 230); // Borders
  const uint16_t colMuted = sprite.color565(240, 240, 240);  // Muted background
  const uint16_t colSuccess = sprite.color565(40, 167, 69);  // #28a745
  const uint16_t colWarning = sprite.color565(255, 193, 7);  // #ffc107 (Yellow)
  const uint16_t colError = sprite.color565(220, 53, 69);    // #dc3545
  const uint16_t colPrimary = sprite.color565(0, 123, 255);  // Active elements
  const uint16_t colTextMuted = sprite.color565(100, 100, 100); // Muted text

  UiKit() {}

  void init();

  void touchUpdate();
  void sidebarTouchUpdate(); // 新しいサイドバーのタッチ処理

  void stmRecvSerial(RobotInfo_t *info, RobotInfo_t *infoPrev);
  void stmSendSerial(RobotInfo_t *info);

  void infoTab(bool forceUpdate = false);

  // フラグ
  bool changeFlag_overMode = true; // モード切り替えのフラグ
  bool homeFlag = false;           // ホーム画面遷移のフラグ

private:
};

#endif
