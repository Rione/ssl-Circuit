#include <Arduino.h>

#include "UI/ui_kit.hpp"
UiKit ui;

#include "UI/unit/touchscreen.h"
XPT2046_Touchscreen ts = XPT2046_Touchscreen(TOUCH_CS);
TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

#include "UI/unit/display.h"
TFT_eSPI tft = TFT_eSPI();  
TFT_eSprite sprite = TFT_eSprite(&tft);
DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

#include <exception>

#include "UI/mode/mainMode.hpp"
MainMode mainMode('M', "Main");

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

UIModeSwitch_t modeData;



void setup() {
  Serial.begin(115200);

  ui.init();

  delay(1000);

  // ui.kickUI();
  // delay(1000);
  
  // ui.topUI();


  modeData.status.mode = 0;
  modeData.modePrev = 0;

  mainMode.displaySet();

}

void loop() {
  ui.touchUpdate();
  mainMode.determine();

  // ui.touchUpdate();


  //タッチ
  // touch.read();

  // //判定
  // if(touch.isTouched && !touch.isTouchedPrev){
  //   if(touch.point.x > 20 && touch.point.x < 140 && touch.point.y > 80 && touch.point.y < 200){
  //     modeData.status.mode = 1;
  //   }
  // }

  // //画面切り替え
  // if(modeData.modePrev != modeData.status.mode){
  //   switch(modeData.status.mode){
  //     case 0:
  //       topUI();
  //       break;
  //     case 1:
  //       kickUI();
  //       break;
  //     default:
  //       break;
  //   }

  //   modeData.modePrev = modeData.status.mode;
  // }

  //
 

  // touch.read();
  

  // kickUI();
  // while(1);
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