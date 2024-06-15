#ifndef __MODE__
#define __MODE__

#include <./unit/Robot.hpp>

class Mode {
  public:
    Mode(char letter, const char name[], Robot *robotPtr) : modeLetter(letter), robot(robotPtr) {
        strcpy(modeName, name);
    }

    char *getModeName() {
        return modeName;
    }

    char getModeLetter() {
        return modeLetter;
    }

    virtual void init() {

    };
    virtual void before() {

    };
    virtual void loop() {

    };
    virtual void after() {

    };

    virtual void encode() {
      
    };
  
    void UiPacketRecv(UIModeSwitch_t *_uiModeSwitchData){
      static const uint8_t HEADER = 0xFF;  // ヘッダ
      static const uint8_t dataSize = 1;  // データのサイズ
      static bool headerReceived = false;  // ヘッダを受信したかどうか
      static uint8_t index = 0;            // 受信したデータのインデックスカウンター
      static uint8_t data[4] = {0}; // 受信したデータ

      while(robot->serial4.available()){
        // 1バイト読み込み
        uint8_t receivedByte = robot->serial4.read();
        printf("get %d\n", receivedByte);
        // printf("received %d\n ", receivedByte);

        if (!headerReceived) {
          index = 0;
          if (receivedByte == HEADER) {
              // ヘッダを受信したらデータの受信を開始
              headerReceived = true; // ヘッダを受信したフラグを立てる
              // printf("header received %d\n ", receivedByte);
          } else {
              headerReceived = false; // ヘッダではないのでフラグをリセット
              // printf("error: Header is not received %d\n", receivedByte);
          }
        } else { // ヘッダを受信した後の処理

          if (index < dataSize) {
            // データ受信
            data[index] = receivedByte;
            // printf("data[%d]: %d\n", index, data[index]);
            index++;

            if (index == dataSize) {   
              // データ受信完了
              _uiModeSwitchData->status.data = data[0];

              index = 0;
                
            }
          }
        }
      }
    }
    
  protected:
    char modeName[24];
    char modeLetter;
    Robot *robot;
    UIModeSwitch_t *uiModeSwitchData;
};

#endif