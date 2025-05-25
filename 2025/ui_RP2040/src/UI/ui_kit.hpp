#ifndef _UI_KIT_
#define _UI_KIT_

#include <Arduino.h>

#include "UI/unit/button.hpp"

#include "UI/font/bold40.h"
#include "UI/font/bold20.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"

#include "Media/MediaExecutor.hpp"

typedef struct {
    // 受け取るデータ
    float batteryVoltage;
    union {
        struct {
            bool chargeState : 1;  // stmから送られてくる充電状態
            uint8_t chargeVol : 7; // capChargeCertitude
        };
        uint8_t data;
    } capaData; 
    uint8_t buzzerState;

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
    uint8_t modePrev = 0;

} RobotInfo_t;

typedef struct {
    uint8_t mode; // モードの状態
    bool emergencyStop; // 緊急停止
    bool charge_state; // 充電状態
    bool kick; // キック状態
} FlagStatus_t;

class UiKit {
  public:
    RobotInfo_t info;
    RobotInfo_t infoPrev;
    FlagStatus_t flagStatus;

    TFT_eSPI tft = TFT_eSPI();
    TFT_eSprite sprite = TFT_eSprite(&tft);
    DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

    XPT2046_Touchscreen ts = XPT2046_Touchscreen(TOUCH_CS);
    TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

    BUTTON main_ChargeButton = BUTTON(&display, &touch, 30, 155, 85, 60, main_chgRedImg, main_chgWhiteImg);
    BUTTON main_KickButton = BUTTON(&display, &touch, 125, 155, 85, 60, main_kickRedImg, main_kickWhiteImg);

    const uint16_t chargeBack = sprite.color565(255, 210, 51); // 充電中の背景色
    const uint16_t tabBack = sprite.color565(195, 216, 242);    // タブの背景色

    UiKit() {
    }

    void init();

    void touchUpdate();
    void homeScreenGesture();

    void stmRecvSerial(RobotInfo_t *info, RobotInfo_t *infoPrev);
    void stmSendSerial(RobotInfo_t *info);

    void infoTab(bool forceUpdate = false);

    bool changeFlag_overMode = true; // モード切り替えのフラグ
    bool homeFlag = false;  // ホーム画面遷移のフラグ

    bool BackLightFlag = true; // true:ON false:OFF
    int BackLightTime = 0;

    int InfoTime = 0;
    bool timeInterval = true; // 電圧の情報出力は一定時間（１秒）ごとに行う

  private:
  
};

#endif
