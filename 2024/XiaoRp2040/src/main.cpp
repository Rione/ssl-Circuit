#include <Arduino.h>

#include "TFT_eSPI.h"
#include "SPI.h"
#include "XPT2046_Touchscreen.h"

#define BackLight 0

XPT2046_Touchscreen ts(TOUCH_CS);
TFT_eSPI tft = TFT_eSPI();  

uint32_t last_time = 0;
bool touch = false;
uint8_t mode = 0;

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

UIModeSwitch_t modeSwitchData;

void ModeSend(uint8_t mode, bool emrg){ 
  static const uint8_t HEADER = 0xFF;
  
  modeSwitchData.status.mode = mode;
  modeSwitchData.status.emergencyStop = emrg;
  modeSwitchData.status.parity = true;

  // uint8_t data = 0b00000100;

  Serial1.write(HEADER);
  Serial1.write(modeSwitchData.status.data);
  Serial.write(modeSwitchData.status.data);
  // Serial.write(data);
}

void topUI(){
  tft.fillRect(0, 0, 160, 240, ILI9341_RED);
  tft.fillRect(160, 0, 160, 240, ILI9341_BLUE);
}

void kickUI(){
  tft.fillRect(0, 0, 160, 240, ILI9341_GREEN);
  tft.fillRect(160, 0, 160, 240, ILI9341_YELLOW);
}

int touchCheck(){
  boolean bTouch = ts.touched();

  if (bTouch == true){
    TS_Point tPoint = ts.getPoint();
    last_time = millis();
    touch = true;
    // Serial.write("touched\n");
    // Serial.printf("(x,y) = (%d, %d)\r\n", tPoint.x, tPoint.y);

    if(tPoint.x < 1800){
      //赤、mainに切り替え
      return 1;
    }else{
      return 2;
    }
  }
  return 0;
}

void kickSend(){

}

void setup() {
  Serial.begin(115200);

  Serial1.setTX(28);
  Serial1.setRX(29);
  Serial1.begin(115200);

  tft.init();
  tft.setRotation(3);

  ts.begin();
  ts.setRotation(1);

  pinMode(BackLight, OUTPUT);
  digitalWrite(BackLight, HIGH);

  topUI();

  modeSwitchData.status.mode = 0;
}

void loop() {
  int touch = touchCheck();

  if(touch == 1){
    //赤
    if(mode != 0){
      //mainに切り替え
      mode = 0;
      // ModeSend(mode, false);
    } 
    tft.fillScreen(ILI9341_RED);
    delay(1000);
    topUI();

  }else if(touch == 2){
    //青
    if(mode != 1){
      //kickに切り替え
      mode = 1;
      // ModeSend(mode, false);
    }
    tft.fillScreen(ILI9341_BLUE);
    kickUI();
    while(touch != 1){
      //黄さらわれたら抜ける
      touch = touchCheck();
      if(touch == 1){
        mode = 0;
        // ModeSend(mode, false);
        topUI();
      }else if(touch == 2){
        // kickSend();
        tft.fillScreen(ILI9341_YELLOW);
        delay(1000);
        kickUI();
      }
    }
    // ModeSend(mode, false);

    // topUI();


  }

  
}


//   // if(Serial1.available() ){
//   //   volt = Serial1.read() / 10.0;

//   //   if(volt != old_volt){

//   //     tft.fillRect(0, 50, 200, 200, ILI9341_BLACK);
//   //     tft.setCursor(20, 50);   
//   //     tft.setTextSize(3); 
//   //     tft.setTextColor(ILI9341_WHITE); 
//   //     tft.print(volt,1);
//   //     tft.println(" v");
//   //     old_volt = volt;
      
//   //   }
   
//   // }



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
