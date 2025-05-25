#include <Arduino.h>

#include "UI/ui_kit.hpp"
UiKit ui;
MediaExecutor media;

#include "Mode/mainMode.hpp"
#include "Mode/home.hpp"
MainMode mainMode('M', "Main", &ui, &media);
Home home('H', "Home", &ui, &media);

Mode *modes[] = {&mainMode};
Mode *currentMode; // 初期モードをmainModeに設定

void modeSwitch() {
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

    currentMode =  modes[ui.info.modeStatus.mode];
    currentMode->displaySet();
}

void loop() {
    static uint32_t time = 0;
    static int interval = 0;
    time = millis();

    ui.touchUpdate();

    currentMode->determine();
    ui.homeScreenGesture();

    modeSwitch();
    currentMode->displaySet();
    
    ui.stmRecvSerial(&ui.info, &ui.infoPrev);    
    ui.infoTab();
    
    media.setBuzzerType((playType)ui.info.buzzerState);

    interval = millis() - time;
    // Serial.println(interval);
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
