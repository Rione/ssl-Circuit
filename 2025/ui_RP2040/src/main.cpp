#include <Arduino.h>

#include "AppManager/ui_kit.hpp"
UiKit ui;
MediaExecutor media;

#include "Mode/sensorMode.hpp"
#include "Mode/testMode.hpp"

SensorMode sensorMode('S', "Sensor", &ui, &media);
TestMode testMode('T', "Test", &ui, &media);

Mode *modes[] = {&sensorMode, &testMode};
Mode *currentMode; // 初期モードをsensorModeに設定

void modeSwitch() {
    if (ui.changeFlag_overMode) {
        Serial.println("modeSwitch");
        // モード番号に従って切り替え (0: Sensor, 1: Test)
        if (ui.info.modeStatus.mode < 2) {
            currentMode = modes[ui.info.modeStatus.mode];
            ui.info.modePrev = ui.info.modeStatus.mode;
        }
    }
}

void setup() {
    Serial.begin(115200);

    ui.init();

    media.init();
    media.playFuncBuzzer(playType::START);

    currentMode =  modes[ui.info.modeStatus.mode];
    currentMode->displaySet();
}

void loop() {
    static uint32_t time = 0;
    static int interval = 0;
    time = millis();

    ui.touchUpdate();

    currentMode->determine();
    ui.sidebarTouchUpdate();

    modeSwitch();
    currentMode->displaySet();
    
    ui.stmRecvSerial(&ui.info, &ui.infoPrev);    
    ui.infoTab();
    
    media.setBuzzerType((playType)ui.info.buzzerState);

    interval = millis() - time;
    // Serial.println(interval);
}

// ブザー再生はcore1で回す。
// core0のloop()内でexecute()すると、setBuzzerType()が毎ループ同じ値を
// 再セットするためエッジ検出(playFuncBuzzer)が働かず、doDirectKick中に
// 一度しか鳴らなくなる。core1で高速ループさせることで、間にNONEを挟んだ
// エッジが繰り返し立ち、「ピコピコ」と鳴り続ける。
void setup1() {
}
void loop1() {
    media.execute();
}

// メモ
//  基本的には縦、横の順で座標を指定する
//  画像の表示はcreateSprite(320, 240)で作成(横、縦)になる
// 　hometabがあるため、画面の座標はfigmaの座標に対して、縦＋30の座標になる
