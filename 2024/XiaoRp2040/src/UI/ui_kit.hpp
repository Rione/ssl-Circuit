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

extern XPT2046_Touchscreen ts;
extern TOUCHSCREEN touch;
extern TFT_eSPI tft;
extern TFT_eSprite sprite;
extern DISPLAY_DEVICE display;

typedef struct {
  union{
      struct{
          uint8_t mode : 6;
          bool emergencyStop : 1;    
          bool parity : 1;      
      };
      uint8_t data;
  }status;

  uint8_t modePrev = 0;

} UIModeSwitch_t;

typedef struct {
  union {
    struct {
      uint8_t mode : 7;
      bool charge: 1;
    };
    uint8_t data;
  }status;
  uint8_t KickPower;
  uint8_t DribblePower;
} KickMode_t;

class UiKit {
  public:
    UIModeSwitch_t modeData;
    KickMode_t kickModeData = {0, 0, 0};

    

    UiKit(){

    }

    void init();

    void touchUpdate();
    void homeScreenGesture();

    bool changeFlag = true;
  
  private:
    

};


#endif
