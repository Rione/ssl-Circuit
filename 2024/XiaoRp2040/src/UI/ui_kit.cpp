#include "ui_kit.hpp"

void UiKit::init(){
  touch.begin();
  display.init();

  display.createSprite();
  display.setBackgroundImage(rione);
  display.publish();
}

void UiKit::touchUpdate(){
  touch.read();
}

void UiKit::topUI(){
  tft.fillScreen(TFT_WHITE);

  display.createSprite(100, 40); //縦、横
  sprite.loadFont(bold40);
  sprite.fillScreen(TFT_WHITE);
  sprite.setTextColor(TFT_BLACK);
  sprite.drawString("Menu", 0, 0);
  display.publish(110, 10);

  display.createSprite(120, 120);
  sprite.loadFont(bold25);
  sprite.setTextColor(TFT_WHITE);
  sprite.fillScreen(TFT_RED);
  sprite.drawString("Kick", 10, 10);
  display.publish(20, 80);

  display.createSprite(120, 120);
  sprite.fillScreen(TFT_BLUE);
  sprite.drawString("Dribble", 10, 10);
  display.publish(180, 80);

}

void UiKit::kickUI(){
  tft.fillScreen(TFT_WHITE);

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

}