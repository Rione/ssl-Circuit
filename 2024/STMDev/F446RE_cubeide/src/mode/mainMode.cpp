#include "mainMode.hpp"

void MainMode::before() {
    // robot->chargeStart();
    robot->led0 = 1;    
    timer.reset();
    printf("mainMode\n");
}

void MainMode::after() {
    robot->led0 = 0;
}

void MainMode::loop() {
    // static int count = 0;
    // count++;
    // timer.reset();
    // if (!robot->info.status.emergencyStop || !robot->info.isUnderVoltage) {
    //     robot->getSensors(&robot->info);
    //     robot->rasSendSerial(robot->info, 8);
    //     robot->rasRecvSerial(); // sync

    //     robot->dribble(robot->info.dribblePower);

    //     // ストレートキックを優先する
    //     if (robot->info.kicker.straight > 0) {
    //         robot->kickStraight(robot->info.kicker.straight);
    //     } else if (robot->info.kicker.chip > 0) {
    //         robot->kickChip(robot->info.kicker.chip);
    //     }

    //     int16_t __velX = meanVelX.calc((float)robot->info.velX.vel);
    //     int16_t __velY = meanVelY.calc((float)robot->info.velY.vel);
    //     int16_t __velAngler = meanVelAngler.calc((float)robot->info.velAngler.vel);
    //     robot->motorDriver.setVelocityFF(__velX, __velY, __velAngler);
    // } else {
    //     // robot->motorDriver.setVelocityFF(0, 0, 0);
    //     robot->motorDriver.sendEmg();
    //     robot->dribble(0);
    // }
    // robot->led1 = robot->info.isHoldBall;
    // printfDMA("Ball:%d Batt:%d\n", robot->info.photoSensorValue, robot->info.batteryVoltage);

    // while (timer.read_us() < 1000) ; // 1ms time control

    if(timer.read_ms() > 100){
        timer.reset();
        robot->led0 = !robot->led0;
    }  
    // UiPacketRecv(robot->uiModeSwitchData, 1);

}

void MainMode::encode(UIModeSwitch_t *_uiModeSwitchData){
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
    

    // printf("mainMode\n"); 
}