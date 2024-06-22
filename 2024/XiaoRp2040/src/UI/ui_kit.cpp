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
