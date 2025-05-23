#include "kickTestMode.hpp"

void KickTestMode::displaySet(UiKit *_ui) {

    if (_ui->changeFlag_overMode) {

        kickUI();
        if (_ui->kickModeData.status.charge == 1) {
            display.setParttImage(130, 120, chargeImg);
            display.publish(20, 90);
        }

        _ui->changeFlag_overMode = false;
    }
}

void KickTestMode::determine(UiKit *_ui) {
    // タッチ
    if (!touch.isTouched && touch.isTouchedPrev) {

        if (touch.point.x > 180 && touch.point.x < 300 && touch.point.y > 70 && touch.point.y < 130 && _ui->kickModeData.status.charge == 1) {
            // straight
            if (_ui->kickModeData.status.mode == 0) {
                display.setParttImage(120, 60, straightRedImg);
                display.publish(180, 70);
                _ui->kickModeData.status.mode = 1;
            } else if (_ui->kickModeData.status.mode == 1) {
                display.setParttImage(120, 60, straightWhiteImg);
                display.publish(180, 70);
                _ui->kickModeData.status.mode = 0;
            }

        } else if (touch.point.x > 180 && touch.point.x < 300 && touch.point.y > 150 && touch.point.y < 210 && _ui->kickModeData.status.charge == 1) {
            // chip
            if (_ui->kickModeData.status.mode == 0) {
                display.setParttImage(120, 60, chipRedImg);
                display.publish(180, 150);
                _ui->kickModeData.status.mode = 2;
            } else if (_ui->kickModeData.status.mode == 2) {
                display.setParttImage(120, 60, chipWhiteImg);
                display.publish(180, 150);
                _ui->kickModeData.status.mode = 0;
            }

        } else if (touch.point.x > 20 && touch.point.x < 150 && touch.point.y > 90 && touch.point.y < 210) {
            // _ui->modeData.status.mode = 0;
            // _ui->changeFlag_overMode = true;

            if (_ui->kickModeData.status.charge == 0) {
                display.setParttImage(130, 120, chargeImg);
                display.publish(20, 90);
                _ui->kickModeData.status.charge = 1;
            } else if (_ui->kickModeData.status.charge == 1) {
                kickUI();
                _ui->kickModeData.status.charge = 0;
            }
        }
    }
}

void KickTestMode::kickUI() {
    display.createSprite(320, 240);
    display.setBackgroundImage(kickImg);
    display.publish();
}