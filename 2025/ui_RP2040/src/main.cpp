#include <Arduino.h>

#include "Media/MediaExecutor.hpp"
MediaExecutor media;

#include "UI/ui_kit.hpp"
UiKit ui;

#include "Mode/mainMode.hpp"
#include "Mode/kickTestMode.hpp"
#include "Mode/home.hpp"
MainMode mainMode('M', "Main", &ui, &media);
KickTestMode kickTestMode('K', "Kick", &ui, &media);
Home home('H', "Home", &ui, &media);

Mode *modes[] = {&mainMode, &kickTestMode};
Mode *currentMode = &mainMode;

void modeSwitch() {
    // if (_ui->modeData.status.mode != _ui->modeData.modePrev) {
    //     currentMode = modes[_ui->modeData.status.mode];
    //     _ui->modeData.modePrev = _ui->modeData.status.mode;
    // }

    if (ui.changeFlag_overMode) {
        Serial.println("modeSwitch");
        if (ui.homeFlag) {
            currentMode = &home;
        } else {
            currentMode = modes[ui.modeData.status.mode];
            ui.modeData.modePrev = ui.modeData.status.mode;
        }
    }
}

void setup() {
    Serial.begin(115200);

    ui.init();

    ui.modeData.status.mode = 0;
    ui.modeData.modePrev = 0;

    currentMode->displaySet();

    ui.BackLightTime = millis();
}

void loop() {
    ui.touchUpdate();

    currentMode->determine();
    modeSwitch();
    currentMode->displaySet();

    ui.stmRecvSerial(&ui.robotInfoData);

    ui.infoTab();

    media.setBuzzerType((playType)ui.robotInfoData.buzzerState);
    // ui.homeScreenGesture();
}

void setup1() {
    media.init();
    media.playFuncBuzzer(playType::START);
    delay(300);
    media.playFuncBuzzer(playType::STOP);
}
void loop1() {
    media.execute();
}

// メモ
//  基本的には縦、横の順で座標を指定する
//  画像の表示はcreateSprite(320, 240)で作成(横、縦)になる
// 　hometabがあるため、画面の座標はfigmaの座標に対して、縦＋30の座標になる
