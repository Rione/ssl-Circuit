#include "ui_kit.hpp"
#include "UI/images/new_image.h"

void UiKit::init() {
    touch.begin();
    display.init();

    // stm縺ｨ縺ｮ騾壻ｿ｡髢句ｧ・
    Serial1.setTX(28);
    Serial1.setRX(29);
    Serial1.begin(115200);

    // 繧ｹ繝励Λ繝・す繝･・郁ｵｷ蜍包ｼ臥判髱｢縺ｮ蜃ｺ蜉・
    display.sprite->fillRect(0, 0, 320, 240, 0xFFFF);
    
    // 320x102縺ｫ譛驕ｩ蛹悶＆繧後◆逕ｻ蜒上ｒ荳ｭ螟ｮ縺ｫ荳諡ｬ謠冗判縺吶ｋ
    display.sprite->pushImage(0, 69, 320, 102, new_image_array);
    
    display.publish(0, 0);

    // 蛻晄悄繝｢繝ｼ繝峨・驕ｸ謚・(0: Sensor, 1: Test)
    info.modeStatus.mode = 0;
    info.modePrev = 0;

    // 繧ｹ繝励Λ繝・す繝･逕ｻ髱｢繧・遘帝俣陦ｨ遉ｺ縺励↑縺後ｉ騾壻ｿ｡蠕・ｩ・
    uint32_t startTime = millis();
    while (millis() - startTime < 2000) {
        stmRecvSerial(&info, &infoPrev);
    }

    // 蛻晄悄UI(繧ｵ繧､繝峨ヰ繝ｼ縲√ヨ繝・・繝舌・)縺ｮ謠冗判
    display.sprite->fillRect(0, 0, 320, 240, 0xFFFF);
    display.publish(); // 蜈ｨ逕ｻ髱｢繧ｯ繝ｪ繧｢

    infoTab(true); // 繝医ャ繝励ヰ繝ｼ繝ｻ繧ｵ繧､繝峨ヰ繝ｼ縺ｮ蛻晏屓譖ｴ譁ｰ
}

void UiKit::touchUpdate() {
    static bool BackLightFlag = true; // 繝舌ャ繧ｯ繝ｩ繧､繝医・繝輔Λ繧ｰ
    static uint32_t BackLightTime = 0;
    
    touch.read();

    if(BackLightTime == 0) BackLightTime = millis();

    if (touch.isTouched) {
        BackLightTime = millis();
        if (!BackLightFlag) {
            tft.writecommand(0x11); // TFT_SLPOUT (Sleep Out)
            delay(120);             // Wake up delay
            tft.writecommand(0x29); // TFT_DISPON (Display On)
            digitalWrite(display.backlightPin, HIGH);
            BackLightFlag = true;
        }
    } else if (BackLightFlag && (millis() - BackLightTime) > 8000) {
        // 8秒間タッチがなければ画面自体をスリープさせ、バックライトも消す
        BackLightFlag = false;
        digitalWrite(display.backlightPin, LOW);
        tft.writecommand(0x28); // TFT_DISPOFF (Display Off)
        tft.writecommand(0x10); // TFT_SLPIN (Sleep In)
    }

}

void UiKit::sidebarTouchUpdate() {
    static bool isTouchedPrev = false;

    if (touch.isTouched && !isTouchedPrev) {
        // Touch is within the sidebar (X < 72)
        if (touch.point.x < 72) {
            // Sensor Tab (Y: 44-108)
            if (touch.point.y >= 44 && touch.point.y <= 108) {
                if (info.modeStatus.mode != 0) {
                    info.modeStatus.mode = 0;
                    changeFlag_overMode = true;
                    media.setBuzzerType(playType::NOTIFY);
                }
            }
            // Test Tab (Y: 116-180)
            else if (touch.point.y >= 116 && touch.point.y <= 180) {
                if (info.modeStatus.mode != 1) {
                    info.modeStatus.mode = 1;
                    changeFlag_overMode = true;
                    media.setBuzzerType(playType::NOTIFY);
                }
            }
            // E-Stop (Y: 188-236)
            else if (touch.point.y >= 188 && touch.point.y <= 236) {
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
    static const uint8_t HEADER = 0xFF;  // 繝倥ャ繝
    static const uint8_t dataSize = 21;  // 諡｡蠑ｵ縺輔ｌ縺溘ョ繝ｼ繧ｿ繧ｵ繧､繧ｺ
    static bool headerReceived = false;  // 繝倥ャ繝繧貞女菫｡縺励◆縺九←縺・°
    static uint8_t index = 0;            // 蜿嶺ｿ｡縺励◆繝・・繧ｿ縺ｮ繧､繝ｳ繝・ャ繧ｯ繧ｹ繧ｫ繧ｦ繝ｳ繧ｿ繝ｼ
    static uint8_t data[dataSize] = {0}; // 蜿嶺ｿ｡縺励◆繝・・繧ｿ

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
                
                // 繝｢繝ｼ繧ｿ繝ｼ騾溷ｺｦ (4繝舌う繝・4)
                memcpy(&info->motor[0].velocity, &data[4], 4);
                memcpy(&info->motor[1].velocity, &data[8], 4);
                memcpy(&info->motor[2].velocity, &data[12], 4);
                memcpy(&info->motor[3].velocity, &data[16], 4);
                
                // 繝｢繝ｼ繧ｿ繝ｼ繧ｹ繝・・繧ｿ繧ｹ (1繝舌う繝医ン繝・ヨ繝槭せ繧ｯ)
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
    // Serial.write(_modeData->status.data); // 繧ｷ繝ｪ繧｢繝ｫ繝・ヰ繝・げ逕ｨ
}

void UiKit::infoTab(bool forceUpdate) {
    // 1. 繝医ャ繝励ヰ繝ｼ (Top Bar) 縺ｮ謠冗判 (繝舌ャ繝・Μ繝ｼ遲・
    static uint32_t timeUpdate = 0;
    if (millis() - timeUpdate > 1000 && info.batteryVoltage != infoPrev.batteryVoltage || forceUpdate) {
        timeUpdate = millis();
        display.sprite->fillRect(72, 0, 248, 36, colBg);
        display.sprite->drawFastHLine(72, 35, 248, colBorder);
        
        display.sprite->setTextColor(colFg, colBg);
        // display.sprite->loadFont(bold20); // 繝輔か繝ｳ繝医・驕ｩ螳・
        display.sprite->setTextDatum(ML_DATUM);
        
        // 繝舌ャ繝・Μ繝ｼ% (莉ｮ險育ｮ・ 16V=100%, 14V=0%)
        int batPct = (info.batteryVoltage - 14.0) * 50;
        if(batPct < 0) batPct = 0;
        if(batPct > 100) batPct = 100;

        // 87% (Bold / 螟ｧ縺阪＞譁・ｭ・
        display.sprite->setTextFont(4); // 26px font
        display.sprite->setTextColor(colFg, colBg);
        String batText = String(batPct) + "%";
        display.sprite->drawString(batText, 80, 18);
        int percentWidth = display.sprite->textWidth(batText);

        // 16.18V (Muted / 蟆上＆縺・枚蟄・
        display.sprite->setTextFont(2); // 16px font
        display.sprite->setTextColor(colTextMuted, colBg);
        String volText = String(info.batteryVoltage, 2) + "V";
        display.sprite->drawString(volText, 80 + percentWidth + 6, 18);
        int volWidth = display.sprite->textWidth(volText);

        // 繝励Ο繧ｰ繝ｬ繧ｹ繝舌・ (濶ｲ蜿ｯ螟・ 隗剃ｸｸ)
        uint16_t barCol = colSuccess; // 繝・ヵ繧ｩ繝ｫ繝医・邱・50%雜・
        if (batPct <= 20) {
            barCol = colError;   // 20%莉･荳・ 襍､
        } else if (batPct <= 50) {
            barCol = colWarning; // 20縲・0%: 鮟・
        }

        int barX = 80 + percentWidth + 6 + volWidth + 12;
        int barW = 310 - barX;
        display.sprite->fillRoundRect(barX, 14, barW, 8, 4, colMuted);
        display.sprite->fillRoundRect(barX, 14, barW * batPct / 100, 8, 4, barCol);
        
        display.publish(0, 0);
    }

    // 2. 繧ｵ繧､繝峨ヰ繝ｼ (Sidebar) 縺ｮ謠冗判
    if (changeFlag_overMode || forceUpdate) {
        display.sprite->fillRect(0, 0, 72, 240, colSidebar);
        display.sprite->drawFastVLine(71, 0, 240, colBorder);
        display.sprite->drawFastHLine(0, 36, 72, colBorder);
        
        display.sprite->setTextDatum(MC_DATUM);
        display.sprite->setTextColor(colFg, colSidebar);
        // display.sprite->setTextFont(2);
        // display.sprite->drawString("Ri-one", 36, 18); // 逕ｻ蜒上′縺ｪ縺・◆繧√ユ繧ｭ繧ｹ繝医〒莉｣逕ｨ
        // 譁ｰ縺励＞繝ｭ繧ｴ繧・70x22 縺ｫ邵ｮ蟆上＠縺ｦ謠冗判 (閭梧勹縺ｮ逋ｽ 0xFFFF 繧帝城℃)
        int logoW = 70;
        int logoH = 22;
        int offsetX = 1;
        int offsetY = 7;
        for (int y = 0; y < logoH; y++) {
            for (int x = 0; x < logoW; x++) {
                int srcX = x * 320 / logoW;
                int srcY = y * 102 / logoH;
                uint16_t color = new_image_array[srcY * 320 + srcX];
                if (color != 0xFFFF) { // 逋ｽ繧帝城℃
                    display.sprite->drawPixel(x + offsetX, y + offsetY, color);
                }
            }
        }
        display.sprite->setTextFont(1);
        
        // Sensor Tab (Larger button)
        if (info.modeStatus.mode == 0) {
            display.sprite->fillRoundRect(4, 44, 64, 64, 4, colPrimary);
        }
        display.sprite->setTextColor(info.modeStatus.mode == 0 ? colBg : colFg);
        display.sprite->drawString("Sensor", 36, 76);
        
        // Test Tab (Larger button)
        if (info.modeStatus.mode == 1) {
            display.sprite->fillRoundRect(4, 116, 64, 64, 4, colPrimary);
        }
        display.sprite->setTextColor(info.modeStatus.mode == 1 ? colBg : colFg);
        display.sprite->drawString("Test", 36, 148);
        
        // E-Stop (邱頑･蛛懈ｭ｢ - at the bottom)
        uint16_t eStopBg = display.sprite->color565(248, 215, 218); // Light red background
        display.sprite->fillRoundRect(4, 188, 64, 48, 4, eStopBg);
        display.sprite->drawRoundRect(4, 188, 64, 48, 4, colError);
        display.sprite->setTextColor(colError);
        display.sprite->drawString("E-Stop", 36, 212);

        display.publish(0, 0);
    }

    if(!forceUpdate) changeFlag_overMode = false;
}
