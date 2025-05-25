#include "mainMode.hpp"

void MainMode::displaySet() {
    // 　モード切替時に画面を変更
    if (ui->changeFlag_overMode) {

        mainUI();

        ui->changeFlag_overMode = false;
    }

    // 　stateの変更
    if (ui->robotInfo.capaData.chargeState != ui->robotInfo.chargeStatePrev) {
        if (ui->robotInfo.capaData.chargeState == 0) {
            ui->display.setParttImage(30, 60, 260, 60, main_dischargeImg);
        } else if (ui->robotInfo.capaData.chargeState == 1) {
            ui->display.setParttImage(30, 60, 260, 60, main_chargeImg);

            ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
            ui->sprite.loadFont(bold25);

            ui->display.createSprite(45, 25);
            ui->sprite.fillSprite(ui->chargeBack);
            ui->sprite.setCursor(0, 0);
            ui->sprite.print(ui->robotInfo.capaData.chargeVol);
            ui->display.publish(187, 80);
        }
    }

    // chargeVolの変更
    if (ui->robotInfo.capaData.chargeVol != ui->robotInfo.chargeVolePrev) {
        ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->chargeBack);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->robotInfo.capaData.chargeVol);
        ui->display.publish(187, 80);
    }
}

void MainMode::determine() {
    // ボタンの判定
    if (ui->main_ChargeButton.determine()) {
        // 送信用にデータを変更
        ui->robotInfo.modeStatus.charge_state = 1;
        ui->stmSendSerial(&ui->robotInfo); // 送信
        ui->robotInfo.modeStatus.charge_state = 0;

        // ブザーを鳴らす
        media->setBuzzerType(NOTIFY);
        
    }else {
        ui->main_ChargeButton.setWhiteImg();
    }

    if(ui->main_KickButton.determine()) {
        // 送信用にデータを変更
        ui->robotInfo.modeStatus.kick = 1;
        ui->stmSendSerial(&ui->robotInfo); // 送信
        ui->robotInfo.modeStatus.kick = 0;

        // ブザーを鳴らす
        media->setBuzzerType(NOTIFY);
    }else {
        ui->main_KickButton.setWhiteImg();
    }
}

void MainMode::mainUI() {
    ui->display.setMainImage(main_BackImg);

    if (ui->robotInfo.capaData.chargeState == 1) {
        ui->display.setParttImage(30, 60, 260, 60, main_chargeImg);

        ui->sprite.setTextColor(TFT_BLACK, ui->chargeBack);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->chargeBack);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->robotInfo.capaData.chargeVol);
        ui->display.publish(187, 80);
    }
}
