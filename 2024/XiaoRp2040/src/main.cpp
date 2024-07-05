#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

#define POWER_PIN 11   // NeoPixelの電源
#define DIN_PIN 12     // NeoPixel　の出力ピン番号
#define LED_COUNT 1    // LEDの連結数
#define WAIT_MS 1000   // 次の点灯までのウエイト
#define BRIGHTNESS 128 // 輝度
Adafruit_NeoPixel pixels(LED_COUNT, DIN_PIN, NEO_GRB + NEO_KHZ800);

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

Mode *modes[] = {&mainMode, &kickTestMode};
Mode *currentMode = &mainMode;

void modeSwitch(UiKit *_ui) {
    if (_ui->modeData.status.mode != _ui->modeData.modePrev) {
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

    currentMode->determine(&ui);
    modeSwitch(&ui);
    currentMode->displaySet(&ui);

    ui.stmRecvSerial(&ui.robotInfoData);

    if (ui.modeData.status.mode != 0) ui.homeScreenGesture();
}

void setup1(){
  //Neopixelの電源供給開始
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  pixels.begin();             //NeoPixel制御開始
}

void loop1() {
  pixels.clear();
  
  //pixels.Color(Red, Green, Blue)で、パレット情報を作成する。
  //赤点灯
  pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, 0, 0));
  pixels.show();
  delay(WAIT_MS);

  //緑点灯
  pixels.setPixelColor(0, pixels.Color(0, BRIGHTNESS, 0));
  pixels.show();
  delay(WAIT_MS);

  //赤　＋　緑　で　黄点灯
  pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, BRIGHTNESS, 0));
  pixels.show();
  delay(WAIT_MS);

  //青点灯
  pixels.setPixelColor(0, pixels.Color(0, 0, BRIGHTNESS));
  pixels.show();
  delay(WAIT_MS);

  //赤　＋　青　で　紫点灯
  pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, 0, BRIGHTNESS));
  pixels.show();
  delay(WAIT_MS);

  //緑　＋　青　で　水点灯
  pixels.setPixelColor(0, pixels.Color(0, BRIGHTNESS, BRIGHTNESS));
  pixels.show();
  delay(WAIT_MS);

  //赤　＋　緑　＋　青　で　白点灯
  pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS));
  pixels.show();
  delay(WAIT_MS);
}

// メモ
//  基本的には縦、横の順で座標を指定する
//  画像の表示はcreateSprite(320, 240)で作成(横、縦)になるu
