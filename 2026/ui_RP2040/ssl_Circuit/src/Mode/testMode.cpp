#include "testMode.hpp"

TestMode::TestMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) 
    : Mode(letter, name, uiPtr, mediaPtr) {
    
    // x, y, w, h, text, bg, fg
    int startX = 80;
    int startY = 48; 
    int gapX = 8;
    int gapY = 8;
    int bw = 110;
    int bh = 56;
    uint16_t btnBg = ui->display.sprite->color565(248, 249, 250); // ui->colSidebar equivalent

    btnDribbler  = new TEXT_BUTTON(&ui->display, &ui->touch, startX,              startY,               bw, bh, "Dribbler",   btnBg, ui->colFg);
    btnKick      = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,  startY,               bw, bh, "Kick",       btnBg, ui->colFg);
    btnChipKick  = new TEXT_BUTTON(&ui->display, &ui->touch, startX,              startY + bh + gapY,   bw, bh, "Chip Kick",  btnBg, ui->colFg);
    btnMotorTest = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,  startY + bh + gapY,   bw, bh, "Motor Test", btnBg, ui->colFg);
    btnCharge    = new TEXT_BUTTON(&ui->display, &ui->touch, startX,              startY + 2*(bh+gapY), bw, bh, "Charge",     btnBg, ui->colFg);
    btnDischarge = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,  startY + 2*(bh+gapY), bw, bh, "Discharge",  btnBg, ui->colFg);
}

void TestMode::displaySet() {
    if (ui->changeFlag_overMode) {
        drawUI();
    }
}

void TestMode::drawUI() {
    ui->display.sprite->fillRect(72, 36, 248, 204, ui->colBg);

    btnDribbler->draw(false);
    btnKick->draw(false);
    btnChipKick->draw(false);
    btnMotorTest->draw(false);
    btnCharge->draw(false);
    btnDischarge->draw(false);

    ui->display.publish(0, 0);
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
