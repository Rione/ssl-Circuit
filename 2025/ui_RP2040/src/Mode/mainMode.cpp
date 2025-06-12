#include "mainMode.hpp"

void MainMode::displaySet() {
    // 　モード切替時に画面を変更
    if (ui->changeFlag_overMode) {
        mainUI();
    }

    // 　stateの変更
    if (ui->info.capaData.chargeState != ui->infoPrev.capaData.chargeState) {
        if (ui->info.capaData.chargeState == 0) {
            ui->display.setParttImage(30, 60, 260, 60, main_dischargeImg);
        } else if (ui->info.capaData.chargeState == 1) {
            ui->display.setParttImage(30, 60, 260, 60, main_chargeImg);

            ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
            ui->sprite.loadFont(bold25);

            ui->display.createSprite(45, 25);
            ui->sprite.fillSprite(ui->chargeBack);
            ui->sprite.setCursor(0, 0);
            ui->sprite.print(ui->info.capaData.chargeVal);
            ui->display.publish(187, 80);
        }
    }

    // chargeValの変更
    if (ui->info.capaData.chargeVal != ui->infoPrev.capaData.chargeVal) {
        ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->chargeBack);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->info.capaData.chargeVal);
        ui->display.publish(187, 80);
    }
}

void MainMode::determine() {
    // ボタンの判定
    if(ui->main_ChargeButton.buttonDisable() || ui->main_KickButton.buttonDisable()) {
        // ボタンが無効化されている場合は何もしない
        ui->main_ChargeButton.setWhiteImg();
        ui->main_KickButton.setWhiteImg();
        return;
    }

    if (ui->main_ChargeButton.determine()) {
        // 送信用にデータを変更
        ui->info.modeStatus.charge_state = 1;
        ui->stmSendSerial(&ui->info); // 送信
        ui->info.modeStatus.charge_state = 0;

        // ブザーを鳴らす
        media->setBuzzerType(NOTIFY);
        
    }else {
        ui->main_ChargeButton.setWhiteImg();
    }

    if(ui->main_KickButton.determine()) {
        // 送信用にデータを変更
        ui->info.modeStatus.kick = 1;
        ui->stmSendSerial(&ui->info); // 送信
        ui->info.modeStatus.kick = 0;

        // ブザーを鳴らす
        media->setBuzzerType(NOTIFY);
    }else {
        ui->main_KickButton.setWhiteImg();
    }
}

void MainMode::mainUI() {
    ui->display.setMainImage(main_BackImg);

    if (ui->info.capaData.chargeState == 1) {
        ui->display.setParttImage(30, 60, 260, 60, main_chargeImg);

        ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->chargeBack);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->info.capaData.chargeVal);
        ui->display.publish(187, 80);
    }
}
