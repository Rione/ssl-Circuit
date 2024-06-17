#include <Arduino.h>

#include "SPI.h"

#include "./unit/display.h"
#include "./unit/touchscreen.h"
#include "font/bold40.h"
#include "font/bold25.h"
#include "font/regular15.h"

#include "image.h"

XPT2046_Touchscreen ts(TOUCH_CS);
TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

TFT_eSPI tft = TFT_eSPI();  
TFT_eSprite sprite = TFT_eSprite(&tft);
DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

void topUI(){
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

void kickUI(){
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


void setup() {
  Serial.begin(115200);

  touch.begin();
  display.init();

  display.createSprite();
  display.setBackgroundImage(rione);
  display.publish();

  delay(1000);

  topUI();

}

void loop() {
  
  

  kickUI();
  while(1);
  // touch.read();

  // if (touch.isTouched) {
  //   display.createSprite();
  //   display.setBackgroundImage(rione);
  //   display.publish();
  // }


  
}


//   //受け取る
//   // if(Serial1.available() ){
//   //   uint8_t data = Serial1.read();
//   //   Serial.print(data);
//   // }

//   //送る
//   // uint8_t data = 100;
//   // Serial1.write(data);
//   // Serial.print(data,DEC);
//   // delay(1000);

// }
