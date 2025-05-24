#include "ui_kit.hpp"

void UiKit::init() {
    touch.begin();
    display.init();

    // stmとの通信開始
    Serial1.setTX(28);
    Serial1.setRX(29);
    Serial1.begin(115200);

    // 起動画面の出力
    display.createSprite();
    display.setBackgroundImage(rione);
    display.publish();

    while (millis() < 3000) {
        stmRecvSerial(&robotInfo);
    }

    // tab部分の出力
    display.setParttImage(320, 30, infoTabImg);
    display.publish();
}

void UiKit::touchUpdate() {
    touch.read();

    if (touch.isTouched) {
        BackLightTime = millis();
        if (!BackLightFlag) {
            digitalWrite(display.backlightPin, HIGH);
            BackLightFlag = true;
        }
    }else if (BackLightFlag && (millis() - BackLightTime) > 15000) {
        // 15秒間タッチがなければバックライトを消す
        BackLightFlag = false;
        digitalWrite(display.backlightPin, LOW);
    }
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
                homeFlag = !homeFlag;
                changeFlag_overMode = true;
                Serial.println("homeFlag");
            }
        }
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
        time = millis();
        timeInterval = true;
    } else if (millis() - time > 1000) {
        timeInterval = false;

        // 電圧の情報出力
        display.createSprite(36, 15);
        sprite.fillScreen(tabBack);
        sprite.setCursor(0, 0);
        sprite.print(robotInfo.batteryVoltage);
        display.publish(281, 8);

        if (robotInfo.batteryVoltage < 14.5) {
            display.setParttImage(16, 16, redCircleImg);
            display.publish(262, 7);
        } else if (robotInfo.batteryVoltage < 16.0) {
            display.setParttImage(16, 16, yellowCircleImg);
            display.publish(262, 7);
        } else {
            display.setParttImage(16, 16, greenCircleImg);
            display.publish(262, 7);
        }
    }

    // コンデンサの情報出力
    display.createSprite(36, 15);
    sprite.fillScreen(tabBack);
    sprite.setCursor(0, 0);
    sprite.print(robotInfo.capaData.chargeVol);
    display.publish(191, 8);

    if (robotInfo.capaData.chargeState == 0) {
        display.setParttImage(16, 16, greenCircleImg);
        display.publish(172, 7);
    } else if (robotInfo.capaData.chargeState == 1) {
        display.setParttImage(16, 16, yellowCircleImg);
        display.publish(172, 7);
    }

    // modeの出力
    switch (robotInfo.modeStatus.mode) {
    case 0: // main
        display.setParttImage(80, 13, Tab_mainImg);
        display.publish(12, 8);
        break;
    case 1: // kick
        display.setParttImage(80, 13, Tab_kickImg);
        display.publish(12, 8);
        break;
    default:
        break;
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
