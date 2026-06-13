#include "testMode.hpp"

TestMode::TestMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) 
    : Mode(letter, name, uiPtr, mediaPtr) {
    
    // x, y, w, h, text, bg, fg
    btnDribbler  = new TEXT_BUTTON(&ui->display, &ui->touch,  76,  32, 118, 44, "Dribbler",   ui->colBg, ui->colFg);
    btnKick      = new TEXT_BUTTON(&ui->display, &ui->touch, 198,  32, 118, 44, "Kick",       ui->colBg, ui->colFg);
    btnChipKick  = new TEXT_BUTTON(&ui->display, &ui->touch,  76,  80, 118, 44, "Chip Kick",  ui->colBg, ui->colFg);
    btnMotorTest = new TEXT_BUTTON(&ui->display, &ui->touch, 198,  80, 118, 44, "Motor Test", ui->colBg, ui->colFg);
    btnCharge    = new TEXT_BUTTON(&ui->display, &ui->touch,  76, 128, 118, 44, "Charge",     ui->colBg, ui->colFg);
    btnDischarge = new TEXT_BUTTON(&ui->display, &ui->touch, 198, 128, 118, 44, "Discharge",  ui->colBg, ui->colFg);
}

void TestMode::displaySet() {
    if (ui->changeFlag_overMode) {
        drawUI();
    }
}

void TestMode::drawUI() {
    ui->display.sprite->fillRect(72, 28, 248, 212, ui->colBg);

    btnDribbler->draw(false);
    btnKick->draw(false);
    btnChipKick->draw(false);
    btnMotorTest->draw(false);
    btnCharge->draw(false);
    btnDischarge->draw(false);

    ui->display.publish(72, 28, 248, 212);
}

void TestMode::determine() {
    // 状態の更新(はなされた時に戻す処理)
    btnDribbler->updateState();
    btnKick->updateState();
    btnChipKick->updateState();
    btnMotorTest->updateState();
    btnCharge->updateState();
    btnDischarge->updateState();

    if (btnDribbler->determine()) {
        media->setBuzzerType(playType::NOTIFY);
        // Add serial send logic later
    }
    if (btnKick->determine()) {
        ui->info.modeStatus.kick = 1;
        ui->stmSendSerial(&ui->info);
        ui->info.modeStatus.kick = 0;
        media->setBuzzerType(playType::NOTIFY);
    }
    if (btnChipKick->determine()) {
        media->setBuzzerType(playType::NOTIFY);
    }
    if (btnMotorTest->determine()) {
        media->setBuzzerType(playType::NOTIFY);
    }
    if (btnCharge->determine()) {
        ui->info.modeStatus.charge_state = 1;
        ui->stmSendSerial(&ui->info);
        ui->info.modeStatus.charge_state = 0;
        media->setBuzzerType(playType::NOTIFY);
    }
    if (btnDischarge->determine()) {
        media->setBuzzerType(playType::NOTIFY);
    }
}
