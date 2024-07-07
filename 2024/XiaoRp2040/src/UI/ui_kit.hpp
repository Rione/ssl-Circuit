#ifndef _UI_KIT_
#define _UI_KIT_

#include "UI/unit/display.h"
#include "UI/unit/touchscreen.h"

#include "UI/font/bold40.h"
#include "UI/font/bold20.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"


#include "UI/image/image.h"
#include "UI/image/top.h"
#include "UI/image/kick.h"
#include "UI/image/main_img.h"
#include "UI/image/home_img.h"

extern XPT2046_Touchscreen ts;
extern TOUCHSCREEN touch;
extern TFT_eSPI tft;
extern TFT_eSprite sprite;
extern DISPLAY_DEVICE display;

static const uint16_t charge_back = sprite.color565(255, 210, 51);  // 充電中の背景色
static const uint16_t tabBack = sprite.color565(195, 216, 242);     // タブの背景色


typedef struct {    
    uint8_t batteryGet;
    union {
        struct {
            bool chargeState : 1; // stmから送られてくる充電状態
            uint8_t chargeVol : 7; //capChargeCertitude
        };
        uint8_t data;

    } capaData;

    float batteryVoltage;
    bool chargeStatePrev;
    uint8_t chargeVolePrev;

} RobotInfo_t; // 受けとるデータ

typedef struct {
    union {
        struct {
            uint8_t mode : 5;
            bool emergencyStop : 1;
            bool charge_state : 1; // 1.切替、0.切替なし
            bool kick : 1;         // キック
        };
        uint8_t data;
    } status;

    uint8_t modePrev = 0;

} UIModeSwitch_t; // main用、一番最初に送るデータ

typedef struct {
    union {
        struct {
            uint8_t mode : 6;
            bool charge : 1;
            bool chargePrev : 1;
        };
        uint8_t data;
    } status;
    uint8_t KickPower;
    uint8_t DribblePower;
} KickMode_t;

class UiKit {
  public:
    RobotInfo_t robotInfoData;
    UIModeSwitch_t modeData;
    KickMode_t kickModeData = {0, 0, 0};

    UiKit() {
    }

    void init();

    void touchUpdate();
    void homeScreenGesture();

    void stmRecvSerial(RobotInfo_t *_robotInfoData);
    void stmSendSerial(UIModeSwitch_t *_modeData);

    void homeTab();

    bool changeFlag_overMode = true;
    bool changeFlag_inMode = false;

    int time = 0;
    bool timeInterval = true; //電圧の情報出力は一定時間（１秒）ごとに行う




  private:
    
};

#endif
