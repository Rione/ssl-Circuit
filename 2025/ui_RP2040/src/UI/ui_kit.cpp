#include "ui_kit.hpp"
#include "UI/image/new_image.h"

void UiKit::init() {
    touch.begin();
    display.init();

    // stmとの通信開始
    Serial1.setTX(28);
    Serial1.setRX(29);
    Serial1.begin(115200);

    // スプラッシュ（起動）画面の出力
    display.sprite->fillScreen(colBg);
    
    // new_image_array is 697x223. Scale or center it.
    // For now, center it. It's wider than 320, so x will be negative.
    display.sprite->pushImage((320 - 697) / 2, (240 - 223) / 2, 697, 223, new_image_array);
    
    display.publish();

    // 初期モードの選択 (0: Sensor, 1: Test)
    info.modeStatus.mode = 0;
    info.modePrev = 0;

    // スプラッシュ画面を2秒間表示しながら通信待機
    uint32_t startTime = millis();
    while (millis() - startTime < 2000) {
        stmRecvSerial(&info, &infoPrev);
    }

    // 初期UI(サイドバー、トップバー)の描画
    display.sprite->fillScreen(colBg);
    display.publish(); // 全画面クリア

    infoTab(true); // トップバー・サイドバーの初回更新
}

void UiKit::touchUpdate() {
    static bool BackLightFlag = true; // バックライトのフラグ
    static uint32_t BackLightTime = 0;
    
    touch.read();

    if(BackLightTime == 0) BackLightTime = millis();

    if (touch.isTouched) {
        BackLightTime = millis();
        if (!BackLightFlag) {
            digitalWrite(display.backlightPin, HIGH);
            BackLightFlag = true;
        }
    }else if (BackLightFlag && (millis() - BackLightTime) > 10000) {
        // 10秒間タッチがなければバックライトを消す
        BackLightFlag = false;
        digitalWrite(display.backlightPin, LOW);
    }

}

void UiKit::sidebarTouchUpdate() {
    static bool isTouchedPrev = false;

    if (touch.isTouched && !isTouchedPrev) {
        // Touch is within the sidebar (X < 72)
        if (touch.point.x < 72) {
            // Sensor Tab (Y: 40-64)
            if (touch.point.y >= 40 && touch.point.y <= 64) {
                if (info.modeStatus.mode != 0) {
                    info.modeStatus.mode = 0;
                    changeFlag_overMode = true;
                    media.setBuzzerType(playType::NOTIFY);
                }
            }
            // Test Tab (Y: 68-92)
            else if (touch.point.y >= 68 && touch.point.y <= 92) {
                if (info.modeStatus.mode != 1) {
                    info.modeStatus.mode = 1;
                    changeFlag_overMode = true;
                    media.setBuzzerType(playType::NOTIFY);
                }
            }
            // E-Stop (Y: 96-120)
            else if (touch.point.y >= 96 && touch.point.y <= 120) {
                info.modeStatus.emergencyStop = 1;
                stmSendSerial(&info);
                info.modeStatus.emergencyStop = 0;
                media.setBuzzerType(playType::SUCCESS); // or alarm
                // Flash background or something
            }
        }
    }
    isTouchedPrev = touch.isTouched;
}

void UiKit::stmRecvSerial(RobotInfo_t *info, RobotInfo_t *infoPrev) {
    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 21;  // 拡張されたデータサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[dataSize] = {0}; // 受信したデータ

    while (Serial1.available()) {
        uint8_t recvData = Serial1.read();

        if (!headerReceived) {
            index = 0;
            if (recvData == HEADER) headerReceived = true;
        } else {
            data[index] = recvData;
            index++;

            if (index == dataSize) {
                headerReceived = false;
                index = 0;
                
                infoPrev->batteryVoltage = info->batteryVoltage;
                infoPrev->capaData.data = info->capaData.data;

                info->batteryVoltage = (float)data[0] / 10.0;
                info->capaData.data = data[1];
                info->buzzerState = data[2];
                info->ballSensor = data[3] > 0;
                
                // モーター速度 (4バイトx4)
                memcpy(&info->motor[0].velocity, &data[4], 4);
                memcpy(&info->motor[1].velocity, &data[8], 4);
                memcpy(&info->motor[2].velocity, &data[12], 4);
                memcpy(&info->motor[3].velocity, &data[16], 4);
                
                // モーターステータス (1バイトビットマスク)
                uint8_t mStatus = data[20];
                info->motor[0].statusOk = (mStatus & 0x01) != 0;
                info->motor[1].statusOk = (mStatus & 0x02) != 0;
                info->motor[2].statusOk = (mStatus & 0x04) != 0;
                info->motor[3].statusOk = (mStatus & 0x08) != 0;
            }
        }
    }
}

void UiKit::stmSendSerial(RobotInfo_t *info) {
    static const uint8_t HEADER = 0xFF;

    Serial1.write(HEADER);
    Serial1.write(info->modeStatus.data);
    // Serial.write(_modeData->status.data); // シリアルデバッグ用
}

void UiKit::infoTab(bool forceUpdate) {
    // 1. トップバー (Top Bar) の描画 (バッテリー等)
    static uint32_t timeUpdate = 0;
    if (millis() - timeUpdate > 1000 && info.batteryVoltage != infoPrev.batteryVoltage || forceUpdate) {
        timeUpdate = millis();
        display.sprite->fillRect(72, 0, 248, 28, colBg);
        display.sprite->drawFastHLine(72, 27, 248, colBorder);
        
        display.sprite->setTextColor(colFg, colBg);
        // display.sprite->loadFont(bold20); // フォントは適宜
        display.sprite->setTextDatum(ML_DATUM);
        
        // バッテリー% (仮計算: 16V=100%, 14V=0%)
        int batPct = (info.batteryVoltage - 14.0) * 50;
        if(batPct < 0) batPct = 0;
        if(batPct > 100) batPct = 100;

        // 87% (Bold / 大きい文字)
        display.sprite->setTextFont(4); // 26px font
        display.sprite->setTextColor(colFg, colBg);
        String batText = String(batPct) + "%";
        display.sprite->drawString(batText, 80, 14);
        int percentWidth = display.sprite->textWidth(batText);

        // 16.18V (Muted / 小さい文字)
        display.sprite->setTextFont(2); // 16px font
        display.sprite->setTextColor(colTextMuted, colBg);
        String volText = String(info.batteryVoltage, 2) + "V";
        display.sprite->drawString(volText, 80 + percentWidth + 6, 14);
        int volWidth = display.sprite->textWidth(volText);

        // プログレスバー (色可変, 角丸)
        uint16_t barCol = colSuccess; // デフォルトは緑(50%超)
        if (batPct <= 20) {
            barCol = colError;   // 20%以下: 赤
        } else if (batPct <= 50) {
            barCol = colWarning; // 20〜50%: 黄
        }

        int barX = 80 + percentWidth + 6 + volWidth + 12;
        int barW = 310 - barX;
        display.sprite->fillRoundRect(barX, 10, barW, 8, 4, colMuted);
        display.sprite->fillRoundRect(barX, 10, barW * batPct / 100, 8, 4, barCol);
        
        display.publish(72, 0, 248, 28);
    }

    // 2. サイドバー (Sidebar) の描画
    if (changeFlag_overMode || forceUpdate) {
        display.sprite->fillRect(0, 0, 72, 240, colSidebar);
        display.sprite->drawFastVLine(71, 0, 240, colBorder);
        display.sprite->drawFastHLine(0, 36, 72, colBorder);
        
        display.sprite->setTextDatum(MC_DATUM);
        display.sprite->setTextColor(colFg, colSidebar);
        display.sprite->drawString("LOGO", 36, 18); // あとで画像に置換
        
        // Sensor Tab
        if (info.modeStatus.mode == 0) display.sprite->fillRect(4, 40, 64, 24, colPrimary);
        display.sprite->setTextColor(info.modeStatus.mode == 0 ? colBg : colFg);
        display.sprite->drawString("Sensor", 36, 52);
        
        // Test Tab
        if (info.modeStatus.mode == 1) display.sprite->fillRect(4, 68, 64, 24, colPrimary);
        display.sprite->setTextColor(info.modeStatus.mode == 1 ? colBg : colFg);
        display.sprite->drawString("Test", 36, 80);
        
        // E-Stop (緊急停止)
        display.sprite->fillRect(4, 96, 64, 24, colMuted);
        display.sprite->drawRect(4, 96, 64, 24, colError);
        display.sprite->setTextColor(colError);
        display.sprite->drawString("E-Stop", 36, 108);

        display.publish(0, 0, 72, 240);
    }

    if(!forceUpdate) changeFlag_overMode = false;
}
