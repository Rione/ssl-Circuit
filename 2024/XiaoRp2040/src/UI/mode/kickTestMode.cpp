#include "kickTestMode.hpp"

void KickTestMode::displaySet(){
  display.displayLight(false);

  display.createSprite(240, 320);
  sprite.fillScreen(TFT_WHITE);
  display.publish();

  display.createSprite(100, 40); //縦、横
  sprite.loadFont(bold40);
  sprite.fillScreen(TFT_WHITE);
  sprite.setTextColor(TFT_BLACK);
  sprite.drawString("Kick", 0, 0);
  display.publish(110, 10);

  display.createSprite(120, 120);
  sprite.loadFont(bold25);
  sprite.setTextColor(TFT_WHITE);
  sprite.fillScreen(TFT_RED);
  sprite.drawString("Main", 10, 10);
  display.publish(20, 80);

  display.createSprite(120, 120);
  sprite.fillScreen(TFT_BLUE);
  sprite.drawString("Shoot", 10, 10);
  display.publish(180, 80);

  display.displayLight(true);
}

void KickTestMode::determine(UiKit *_ui){
  //タッチ
  if(touch.isTouched && !touch.isTouchedPrev){
    if(touch.point.x > 20 && touch.point.x < 140 && touch.point.y > 80 && touch.point.y < 200){

      _ui->modeData.status.mode = 0;

    }
  }
}