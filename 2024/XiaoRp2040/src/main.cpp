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
SPISettings settings(120000000, MSBFIRST, SPI_MODE0);

uint32_t last_time = 0;
bool touch = false;
uint8_t mode = 0;

// float volt, old_volt = 0.0;

typedef struct {
    union{
        struct{
            uint8_t mode : 6;
            bool emergencyStop : 1;          
            bool parity : 1;
        };
        uint8_t data;
    }status;

    bool modePrev = 0;

} UIModeSwitch_t;

UIModeSwitch_t modeSwitchData;

void ModeSend(uint8_t mode, bool emrg){ 
  static const uint8_t HEADER = 0xFF;
  
  modeSwitchData.status.mode = mode;
  modeSwitchData.status.emergencyStop = emrg;

  // data = 0b00000100;dd

  Serial1.write(HEADER);
  Serial1.write(modeSwitchData.status.data);
  Serial.write(modeSwitchData.status.data);
}



void setup(){
  Serial.begin(115200);

  Serial1.setTX(28);            
  Serial1.setRX(29);  
  Serial1.begin(115200); 

  //ESP SPI ピン設定
  SPI.setTX(COMMON_MOSI);
  SPI.setRX(COMMON_MISO);
  SPI.setSCK(COMMON_SCK);

  //表示開始
  tft.begin();
  tft.setRotation(3);
  tft.fillRect(0, 0, 160, 240, ILI9341_YELLOW);
  tft.fillRect(160, 0, 160, 240, ILI9341_BLUE);

  // タッチ入力開始
  ts.begin();
  ts.setRotation(1);

  pinMode(light, OUTPUT); 
  digitalWrite(light, HIGH);

  SPI.beginTransaction(settings);
}



void loop(){
  boolean bTouch = ts.touched();

  if (bTouch == true){
    TS_Point tPoint = ts.getPoint();
    last_time = millis();
    touch = true;
    // Serial.printf("(x,y) = (%d, %d)\r\n", tPoint.x, tPoint.y);
    if(tPoint.x < 1800){
      tft.fillScreen(ILI9341_RED);
      if(mode != 0){
        mode = 0;
        ModeSend(mode, false);
      }
      
    }else{
      tft.fillScreen(ILI9341_BLUE);
      if(mode != 1){
        mode = 1;
        ModeSend(mode, false);
      }
    }
  }

  if(millis()-last_time > 1000 && touch == true){
    tft.fillRect(0, 0, 160, 240, ILI9341_RED);
    tft.fillRect(160, 0, 160, 240, ILI9341_BLUE);
    touch = false;
  }
   


  // if(Serial1.available() ){
  //   volt = Serial1.read() / 10.0;

  //   if(volt != old_volt){

  //     tft.fillRect(0, 50, 200, 200, ILI9341_BLACK);
  //     tft.setCursor(20, 50);   
  //     tft.setTextSize(3); 
  //     tft.setTextColor(ILI9341_WHITE); 
  //     tft.print(volt,1);
  //     tft.println(" v");
  //     old_volt = volt;
      
  //   }
   
  // }



  //受け取る
  // if(Serial1.available() ){
  //   uint8_t data = Serial1.read();
  //   Serial.print(data);
  // }

  //送る
  // uint8_t data = 100;
  // Serial1.write(data);
  // Serial.print(data,DEC);
  // delay(1000);

  // delay(1000);

}
