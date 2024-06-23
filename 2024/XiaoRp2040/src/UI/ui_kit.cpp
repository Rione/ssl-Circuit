#include "ui_kit.hpp"

void UiKit::init(){
  touch.begin();
  display.init();

  display.displayLight(false);

  display.createSprite();
  display.setBackgroundImage(rione);
  display.publish();

  display.displayLight(true);
}

void UiKit::touchUpdate(){
  touch.read();
}

void UiKit::homeScreenGesture() {
  static bool flag = false;

  static int yWhenFlagged = 0;
  static unsigned long timeWhenFlagged = 0;

  if (touch.isTouched) {
    if (abs(2000 - touch.raw.x) < 800 && touch.raw.y > 3000 && !touch.isTouchedPrev) {
      flag = true;
      yWhenFlagged = touch.raw.y;
      timeWhenFlagged = millis();
    }

    if (flag && millis() - timeWhenFlagged < 200) {
      if (touch.raw.y < yWhenFlagged - 300) {
        flag = false;
        modeData.status.mode = 0;
        changeFlag = true;
      }
    }
  }
}
