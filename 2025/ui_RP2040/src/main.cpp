#include <Arduino.h>

#include "UI/ui_kit.hpp"
UiKit ui;
MediaExecutor media;

#include "Mode/mainMode.hpp"
#include "Mode/home.hpp"
MainMode mainMode('M', "Main", &ui, &media);
Home home('H', "Home", &ui, &media);

Mode *modes[] = {&mainMode};
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
            currentMode = modes[ui.info.modeStatus.mode];
            ui.info.modePrev = ui.info.modeStatus.mode;
        }
    }
}

void setup() {
    Serial.begin(115200);

    ui.init();

    ui.info.modeStatus.mode = 0;
    ui.info.modePrev = 0;

    currentMode->displaySet();

    ui.BackLightTime = millis();


}

void loop() {
    ui.touchUpdate();

    currentMode->determine();
    modeSwitch();
    currentMode->displaySet();

    ui.homeScreenGesture();
    ui.stmRecvSerial(&ui.info, &ui.infoPrev);    
    ui.infoTab();
    
    media.setBuzzerType((playType)ui.info.buzzerState);
    
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
