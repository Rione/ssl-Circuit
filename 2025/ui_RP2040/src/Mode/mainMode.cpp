#include "mainMode.hpp"

void MainMode::displaySet() {
    // 　モード切替時に画面を変更
    if (ui->changeFlag_overMode) {

        mainUI();

        ui->changeFlag_overMode = false;
    }

    // 　ボタンが押された時の処理
    if (isTouched_state && millis() - isTouchedTime > 1000) {
        ui->display.setParttImage(85, 60, main_chgWhiteImg);
        ui->display.publish(30, 155);
        isTouched_state = false;
    } else if (isTouched_kick && millis() - isTouchedTime > 1000) {
        ui->display.setParttImage(85, 60, main_kickWhiteImg);
        ui->display.publish(125, 155);
        isTouched_kick = false;
    }

    // 　stateの変更
    if (ui->robotInfo.capaData.chargeState != ui->robotInfo.chargeStatePrev) {
        if (ui->robotInfo.capaData.chargeState == 0) {
            ui->display.setParttImage(260, 60, main_dischargeImg);
            ui->display.publish(30, 60);
        } else if (ui->robotInfo.capaData.chargeState == 1) {
            ui->display.setParttImage(260, 60, main_chargeImg);
            ui->display.publish(30, 60);

            ui->sprite.setTextColor(TFT_BLACK, ui->charge_back);
            ui->sprite.loadFont(bold25);

            ui->display.createSprite(45, 25);
            ui->sprite.fillSprite(ui->charge_back);
            ui->sprite.setCursor(0, 0);
            ui->sprite.print(ui->robotInfo.capaData.chargeVol);
            ui->display.publish(187, 80);
        }
    }

    // chargeVolの変更
    if (ui->robotInfo.capaData.chargeVol != ui->robotInfo.chargeVolePrev) {
        ui->sprite.setTextColor(TFT_BLACK, ui->charge_back);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->charge_back);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->robotInfo.capaData.chargeVol);
        ui->display.publish(187, 80);
    }
}

void MainMode::determine() {
    // タッチ
    if (!ui->touch.isTouched && ui->touch.isTouchedPrev && !isTouched_state && !isTouched_kick) {
        if (ui->touch.point.x > 30 && ui->touch.point.x < 115 && ui->touch.point.y > 155 && ui->touch.point.y < 215) {

            // 画面赤く
            ui->display.setParttImage(85, 60, main_chgRedImg);
            ui->display.publish(30, 155);
            // タッチした時間を記録
            isTouched_state = true;
            isTouchedTime = millis();

            // 送信用にデータを変更
            ui->robotInfo.modeStatus.charge_state = 1;
            ui->stmSendSerial(&ui->robotInfo); // 送信
            ui->robotInfo.modeStatus.charge_state = 0;

            // ブザーを鳴らす
            media->setBuzzerType(NOTIFY);

        } else if (ui->touch.point.x > 125 && ui->touch.point.x < 210 && ui->touch.point.y > 155 && ui->touch.point.y < 215) {

            ui->display.setParttImage(85, 60, main_kickRedImg);
            ui->display.publish(125, 155);

            isTouched_kick = true;
            isTouchedTime = millis();

            // 送信用にデータを変更
            ui->robotInfo.modeStatus.kick = 1;
            ui->stmSendSerial(&ui->robotInfo); // 送信
            ui->robotInfo.modeStatus.kick = 0;

            // ブザーを鳴らす
            media->setBuzzerType(NOTIFY);
        }
    }
}

void MainMode::mainUI() {
    ui->display.setParttImage(320, 210, main_BackImg);
    ui->display.publish(0, 30);

    if (ui->robotInfo.capaData.chargeState == 1) {
        ui->display.setParttImage(260, 60, main_dischargeImg);
        ui->display.publish(30, 60);

        ui->display.setParttImage(260, 60, main_chargeImg);
        ui->display.publish(30, 60);

        ui->sprite.setTextColor(TFT_BLACK, ui->charge_back);
        ui->sprite.loadFont(bold25);

        ui->display.createSprite(45, 25);
        ui->sprite.fillSprite(ui->charge_back);
        ui->sprite.setCursor(0, 0);
        ui->sprite.print(ui->robotInfo.capaData.chargeVol);
        ui->display.publish(187, 80);
    }
}
