#include "ui_kit.hpp"

void UiKit::init() {
    touch.begin();
    display.init();

    // stmとの通信開始
    Serial1.setTX(28);
    Serial1.setRX(29);
    Serial1.begin(115200);

    // 起動画面の出力
    display.setBackgroundImage(rione);

    while (millis() < 3000) {
        stmRecvSerial(&robotInfo);
    }

    // tab部分の出力
    display.setParttImage(0, 0, 320, 30, infoTabImg);
}

void UiKit::touchUpdate() {
    touch.read();

    if (touch.isTouched) {
        BackLightTime = millis();
        if (!BackLightFlag) {
            digitalWrite(display.backlightPin, HIGH);
            BackLightFlag = true;
        }
    }else if (BackLightFlag && (millis() - BackLightTime) > 10000) {
        // 15秒間タッチがなければバックライトを消す
        BackLightFlag = false;
        digitalWrite(display.backlightPin, LOW);
    }

    
}

void UiKit::homeScreenGesture() {
    static bool flag = false;
    static unsigned long timeWhenFlagged = 0;

    if (touch.isTouched) {
        if (touch.point.y < 60 && !touch.isTouchedPrev && !flag) {
            // Serial.println("flagged");
            flag = true;
            timeWhenFlagged = millis();
        }else if (flag && (millis() - timeWhenFlagged) > 400) {
            if (millis() - timeWhenFlagged < 600) {
                media.setBuzzerType(playType::NOTIFY);
            } else {
                media.setBuzzerType(playType::SUCCESS);
                flag = false;
                homeFlag = !homeFlag;
                changeFlag_overMode = true;
                // Serial.println("homeFlag");
            }
        }
    } else {
        flag = false;
    }
}

void UiKit::stmRecvSerial(RobotInfo_t *robotInfo) {
    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 3;   // データのサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[dataSize] = {0}; // 受信したデータ

    while (Serial1.available()) {
        uint8_t recvData = Serial1.read();
        // Serial.write(recvData); // シリアルデバッグ用

        if (!headerReceived) {
            index = 0;
            if (recvData == HEADER) headerReceived = true;
        } else {
            data[index] = recvData;
            index++;

            if (index == dataSize) {
                headerReceived = false;
                index = 0;

                robotInfo->chargeStatePrev = robotInfo->capaData.chargeState;
                robotInfo->chargeVolePrev = robotInfo->capaData.chargeVol;

                robotInfo->batteryGet = data[0];
                robotInfo->capaData.data = data[1];
                robotInfo->buzzerState = data[2];

                robotInfo->batteryVoltage = (float)robotInfo->batteryGet / 10.0;
            }
        }
    }
}

void UiKit::stmSendSerial(RobotInfo_t *robotInfo) {
    static const uint8_t HEADER = 0xFF;

    Serial1.write(HEADER);
    Serial1.write(robotInfo->modeStatus.data);
    // Serial.write(_modeData->status.data); // シリアルデバッグ用
}

void UiKit::infoTab() {
    sprite.setTextColor(TFT_BLACK, tabBack);
    sprite.loadFont(regular15);

    // 電圧は１秒に１回のみ出力
    if (!timeInterval) {
        InfoTime = millis();
        timeInterval = true;
    } else if (millis() - InfoTime > 1000) {
        timeInterval = false;

        // 電圧の情報出力
        display.createSprite(36, 15);
        sprite.fillScreen(tabBack);
        sprite.setCursor(0, 0);
        sprite.print(robotInfo.batteryVoltage);
        display.publish(281, 8);

        if (robotInfo.batteryVoltage < 14.5) {
            display.setParttImage(262, 7, 16, 16, redCircleImg);
        } else if (robotInfo.batteryVoltage < 16.0) {
            display.setParttImage(262, 7, 16, 16, yellowCircleImg);
        } else {
            display.setParttImage(262, 7, 16, 16, greenCircleImg);
        }
    }

    // コンデンサの情報出力
    display.createSprite(36, 15);
    sprite.fillScreen(tabBack);
    sprite.setCursor(0, 0);
    sprite.print(robotInfo.capaData.chargeVol);
    display.publish(191, 8);

    if (robotInfo.capaData.chargeState == 0) {
        display.setParttImage(172, 7, 16, 16, greenCircleImg);
    } else if (robotInfo.capaData.chargeState == 1) {
        display.setParttImage(172, 7, 16, 16, yellowCircleImg);
    }

    // modeの出力
    if(changeFlag_overMode){
        switch (robotInfo.modeStatus.mode) {
        case 0: // main
            display.setParttImage(12, 8, 80, 13, Tab_mainImg);
            break;
        case 1: // kick
            display.setParttImage(12, 8, 80, 13, Tab_kickImg);
            break;
        default:
            break;
        }
    }
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
