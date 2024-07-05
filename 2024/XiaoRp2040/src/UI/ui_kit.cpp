#include "ui_kit.hpp"

void UiKit::init(){
  touch.begin();
  display.init();

  display.createSprite();
  display.setBackgroundImage(rione);
  display.publish();


  Serial1.setTX(28);
  Serial1.setRX(29);
  Serial1.begin(115200);

}

void UiKit::touchUpdate(){
  touch.read();
}

void UiKit::homeScreenGesture() {
  static bool flag = false;

  static int yWhenFlagged = 0;
  static unsigned long timeWhenFlagged = 0;

  if (touch.isTouched) {
    if (abs(2000 - touch.raw.x) < 800 && touch.raw.y > 3000 && !touch.isTouchedPrev) {
      flag = true;
      yWhenFlagged = touch.raw.y;
      timeWhenFlagged = millis();
    }

    if (flag && millis() - timeWhenFlagged < 200) {
      if (touch.raw.y < yWhenFlagged - 300) {
        flag = false;
        modeData.status.mode = 0;
        changeFlag_overMode = true;
      }
    }
  }
}

void UiKit::stmRecvSerial(RobotInfo_t *_robotInfoData){
  _robotInfoData->chargePrev = _robotInfoData->status.charge;
  // _robotInfoData->chargePrev = 0;

  if(Serial1.available()){
    uint8_t recvData = Serial1.read();
    Serial.print(recvData);
    // Serial1.print(recvData);
    _robotInfoData->status.data = recvData;
    // _robotInfoData->status.charge = 1;
  }
}

void UiKit::stmSendSerial(UIModeSwitch_t *_modeData){
  static const uint8_t HEADER = 0xFF;

  // Serial1.write(HEADER);
  // Serial1.write(_modeData->status.data);
  
  // Serial.write(HEADER);
  Serial1.write(_modeData->status.data);
  Serial.write(_modeData->status.data);
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
