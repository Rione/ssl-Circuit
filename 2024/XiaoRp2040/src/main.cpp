#include <Arduino.h>

#include "Media/MediaExecutor.hpp"
MediaExecutor media;

#include "UI/ui_kit.hpp"
UiKit ui;

#include "UI/unit/touchscreen.h"
XPT2046_Touchscreen ts = XPT2046_Touchscreen(TOUCH_CS);
TOUCHSCREEN touch = TOUCHSCREEN(&ts, TOUCH_CS);

#include "UI/unit/display.h"
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
DISPLAY_DEVICE display = DISPLAY_DEVICE(&tft, &sprite);

#include <exception>

#include "UI/mode/mainMode.hpp"
#include "UI/mode/kickTestMode.hpp"
MainMode mainMode('M', "Main", &ui, &media);
KickTestMode kickTestMode('K', "Kick", &ui, &media);

Mode *modes[] = {&mainMode, &kickTestMode};
Mode *currentMode = &mainMode;

void modeSwitch(UiKit *_ui) {
    if (_ui->modeData.status.mode != _ui->modeData.modePrev) {
        currentMode = modes[_ui->modeData.status.mode];
        _ui->modeData.modePrev = _ui->modeData.status.mode;
    }
}

void setup() {
    Serial.begin(115200);

    ui.init();

    ui.modeData.status.mode = 0;
    ui.modeData.modePrev = 0;

    currentMode->displaySet(&ui);

    // ui.robotInfoData.capaData.chargeState = 0;
    // ui.robotInfoData.capaData.chargeVol = 0;
    // ui.robotInfoData.batteryVoltage = 15.0;
}

void loop() {

    ui.touchUpdate();

    currentMode->determine(&ui);
    modeSwitch(&ui);
    currentMode->displaySet(&ui);

    ui.stmRecvSerial(&ui.robotInfoData);

    ui.infoTab();

    if (ui.modeData.status.mode != 0) ui.homeScreenGesture();
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
