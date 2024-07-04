#ifndef _UI_KIT_
#define _UI_KIT_


#include "UI/unit/display.h"
#include "UI/unit/touchscreen.h"

#include "UI/font/bold40.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"

#include "UI/image/image.h"
#include "UI/image/top.h"
#include "UI/image/kick.h"
#include "UI/image/main_kick.h"

extern XPT2046_Touchscreen ts;
extern TOUCHSCREEN touch;
extern TFT_eSPI tft;
extern TFT_eSprite sprite;
extern DISPLAY_DEVICE display;

typedef struct {
  union{
    struct{
      bool charge : 1; //stmから送られてくる充電状態
      uint8_t reserve : 7; 
    };
    uint8_t data;

  }status;

  bool chargePrev;  
  
} RobotInfo_t; //受けとるデータ

typedef struct {
  union{
    struct{
      uint8_t mode : 5;
      bool emergencyStop : 1;  
      bool charge_state : 1;   //1.切替、0.切替なし
      bool kick : 1;           //キック
    };
    uint8_t data;
  }status;

  uint8_t modePrev = 0;

} UIModeSwitch_t; //main用、一番最初に送るデータ

typedef struct {
  union {
    struct {
      uint8_t mode : 6;
      bool charge: 1;
      bool chargePrev: 1;

    };
    uint8_t data;
  }status;
  uint8_t KickPower;
  uint8_t DribblePower;
} KickMode_t;

class UiKit {
  public:
    RobotInfo_t robotInfoData;
    UIModeSwitch_t modeData;
    KickMode_t kickModeData = {0, 0, 0};

    

    UiKit(){

    }

    void init();

    void touchUpdate();
    void homeScreenGesture();

    void stmRecvSerial(RobotInfo_t *_robotInfoData);
    void stmSendSerial(UIModeSwitch_t *_modeData);

    bool changeFlag_overMode = true;
    bool changeFlag_inMode = false;

    bool sendFlag = false;
  
  private:
    

};


#endif
