#include "kickTestMode.hpp"

void KickTestMode::before() {
    robot->led1 = 1;
    timer.reset();
    printf("kickTestMode\n");
}   

void KickTestMode::after() {
    robot->led1 = 0;
   
}

void KickTestMode::loop(){
    if(timer.read_ms() > 100){
        timer.reset();
        robot->led1 = !robot->led1;
    }
    // UiPacketRecv(uiModeSwitchData, 1);
    
}

void KickTestMode::encode(UIModeSwitch_t *_uiModeSwitchData){
    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 1;  // データのサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[4] = {0}; // 受信したデータ

    while(robot->serial4.available()){
        // 1バイト読み込み
        uint8_t receivedByte = robot->serial4.read();
        // printf("get %d\n", receivedByte);

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
                    headerReceived = false;
                    
                }
            }
        }
    }
    

    // printf("kickTestMode\n");
    
}