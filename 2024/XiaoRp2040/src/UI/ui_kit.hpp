#ifndef _UI_KIT_
#define _UI_KIT_

#include "UI/unit/display.h"
#include "UI/unit/touchscreen.h"

#include "UI/font/bold40.h"
#include "UI/font/bold25.h"
#include "UI/font/regular15.h"

#include "UI/image/image.h"
#include "UI/image/top.h"

class UiKit {
  public:
    UiKit(){

    }

    void init();

    void topUI();
    void kickUI();
  
  private:
    XPT2046_Touchscreen ts = XPT2046_Touchscreen(TOUCH_CS);
    TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

    TFT_eSPI tft = TFT_eSPI();  
    TFT_eSprite sprite = TFT_eSprite(&tft);
    DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

};


#endif
