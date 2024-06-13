#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>


#define COMMON_SCK  2
#define COMMON_MOSI 3
#define COMMON_MISO 4

#define TOUCH_CS 6

#define TFT_DC  27
#define TFT_CS  1
#define TFT_RST 26

#define light 0

#define spk 7

XPT2046_Touchscreen ts(TOUCH_CS);
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

void setup()
{
  Serial.begin(115200);

  //ESP SPI ピン設定
  SPI.setTX(COMMON_MOSI);
  SPI.setRX(COMMON_MISO);
  SPI.setSCK(COMMON_SCK);

  //表示開始
  tft.begin();
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.fillScreen(ILI9341_RED);

  //タッチ入力開始
  ts.begin();
  ts.setRotation(3);

  pinMode(light, OUTPUT); 
  digitalWrite(light, LOW);

}



void loop()
{
  boolean bTouch = ts.touched();

  if (bTouch == true)
  {
   digitalWrite(light, HIGH);
   delay(3000);
   digitalWrite(light, LOW);
  }
   
}
