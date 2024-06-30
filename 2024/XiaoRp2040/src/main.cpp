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
#include "UI/mode/kickTestMode.hpp"
MainMode mainMode('M', "Main", &ui);
KickTestMode kickTestMode('K', "Kick", &ui);

Mode *modes[] = {&mainMode,&kickTestMode};
Mode *currentMode = &mainMode;

void modeSwitch(UiKit *_ui){
  if(_ui->modeData.status.mode != _ui->modeData.modePrev){
    currentMode = modes[_ui->modeData.status.mode];
    _ui->modeData.modePrev = _ui->modeData.status.mode;
  }
}

void setup() {
  Serial.begin(115200);

  ui.init();

  delay(1000);

  ui.modeData.status.mode = 0;
  ui.modeData.modePrev = 0;

  currentMode->displaySet(&ui);

}

void loop() {
  ui.touchUpdate();
  if(!touch.isTouched && touch.isTouchedPrev){
    currentMode->determine(&ui);
    modeSwitch(&ui);
    currentMode->displaySet(&ui);
  }
  if(ui.modeData.status.mode != 0) ui.homeScreenGesture();
  
  
  
}

//メモ
// 基本的には縦、横の順で座標を指定する
// 画像の表示はcreateSprite(320, 240)で作成(横、縦)になるu


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