#ifndef _UI_KIT_
#define _UI_KIT_

#include "UI/unit/display.h"
#include "UI/unit/touchscreen.h"

#include "UI/font/bold40.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"

#include "UI/image/image.h"
#include "UI/image/top.h"

extern XPT2046_Touchscreen ts;
extern TOUCHSCREEN touch;
extern TFT_eSPI tft;
extern TFT_eSprite sprite;
extern DISPLAY_DEVICE display;

class UiKit {
  public:
    UiKit(){

    }

    void init();

    void touchUpdate();

    void topUI();
    void kickUI();
  
  private:
    

};


#endif
