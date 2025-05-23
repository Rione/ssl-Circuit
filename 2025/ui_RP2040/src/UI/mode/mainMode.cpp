#include "mainMode.hpp"

void MainMode::displaySet(UiKit *_ui) {
    // 　モード切替時に画面を変更
    if (_ui->changeFlag_overMode) {

        mainUI(_ui);

        _ui->changeFlag_overMode = false;
    }

    // 　ボタンが押された時の処理
    if (isTouched_state && millis() - isTouchedTime > 1000) {
        display.setParttImage(85, 60, main_chgWhiteImg);
        display.publish(30, 155);
        isTouched_state = false;
    } else if (isTouched_kick && millis() - isTouchedTime > 1000) {
        display.setParttImage(85, 60, main_kickWhiteImg);
        display.publish(125, 155);
        isTouched_kick = false;
    }

    // 　stateの変更
    if (_ui->robotInfoData.capaData.chargeState != _ui->robotInfoData.chargeStatePrev) {
        if (_ui->robotInfoData.capaData.chargeState == 0) {
            display.setParttImage(260, 60, main_dischargeImg);
            display.publish(30, 60);
        } else if (_ui->robotInfoData.capaData.chargeState == 1) {
            display.setParttImage(260, 60, main_chargeImg);
            display.publish(30, 60);

            sprite.setTextColor(TFT_BLACK, charge_back);
            sprite.loadFont(bold25);

            display.createSprite(45, 25);
            sprite.fillSprite(charge_back);
            sprite.setCursor(0, 0);
            sprite.print(_ui->robotInfoData.capaData.chargeVol);
            display.publish(187, 80);
        }
    }

    // chargeVolの変更
    if (_ui->robotInfoData.capaData.chargeVol != _ui->robotInfoData.chargeVolePrev) {
        sprite.setTextColor(TFT_BLACK, charge_back);
        sprite.loadFont(bold25);

        display.createSprite(45, 25);
        sprite.fillSprite(charge_back);
        sprite.setCursor(0, 0);
        sprite.print(_ui->robotInfoData.capaData.chargeVol);
        display.publish(187, 80);
    }
}

void MainMode::determine(UiKit *_ui) {
    // タッチ
    if (!touch.isTouched && touch.isTouchedPrev && !isTouched_state && !isTouched_kick) {
        if (touch.point.x > 30 && touch.point.x < 115 && touch.point.y > 155 && touch.point.y < 215) {

            // 画面赤く
            display.setParttImage(85, 60, main_chgRedImg);
            display.publish(30, 155);
            // タッチした時間を記録
            isTouched_state = true;
            isTouchedTime = millis();

            // 送信用にデータを変更
            _ui->modeData.status.charge_state = 1;
            _ui->stmSendSerial(&_ui->modeData); // 送信
            _ui->modeData.status.charge_state = 0;

            // ブザーを鳴らす
            media->setBuzzerType(NOTIFY);

        } else if (touch.point.x > 125 && touch.point.x < 210 && touch.point.y > 155 && touch.point.y < 215) {

            display.setParttImage(85, 60, main_kickRedImg);
            display.publish(125, 155);

            isTouched_kick = true;
            isTouchedTime = millis();

            // 送信用にデータを変更
            _ui->modeData.status.kick = 1;
            _ui->stmSendSerial(&_ui->modeData); // 送信
            _ui->modeData.status.kick = 0;

            // ブザーを鳴らす
            media->setBuzzerType(NOTIFY);
        }
    }
}

void MainMode::mainUI(UiKit *_ui) {
    display.setParttImage(320, 210, main_BackImg);
    display.publish(0, 30);

    if (_ui->robotInfoData.capaData.chargeState == 1) {
        display.setParttImage(260, 60, main_dischargeImg);
        display.publish(30, 60);

        display.setParttImage(260, 60, main_chargeImg);
        display.publish(30, 60);

        sprite.setTextColor(TFT_BLACK, charge_back);
        sprite.loadFont(bold25);

        display.createSprite(45, 25);
        sprite.fillSprite(charge_back);
        sprite.setCursor(0, 0);
        sprite.print(_ui->robotInfoData.capaData.chargeVol);
        display.publish(187, 80);
    }
}
