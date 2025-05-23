#include "kickTestMode.hpp"

void KickTestMode::displaySet() {

    if (ui->changeFlag_overMode) {

        kickUI();

        if (ui->kickModeData.status.charge == 1) {
            ui->display.setParttImage(130, 120, chargeImg);
            ui->display.publish(20, 90);
        }

        ui->changeFlag_overMode = false;
    }
}

void KickTestMode::determine() {
    // タッチ
    if (!ui->touch.isTouched && ui->touch.isTouchedPrev) {

        if (ui->touch.point.x > 180 && ui->touch.point.x < 300 && ui->touch.point.y > 70 && ui->touch.point.y < 130 && ui->kickModeData.status.charge == 1) {
            // straight
            if (ui->kickModeData.status.mode == 0) {
                ui->display.setParttImage(120, 60, straightRedImg);
                ui->display.publish(180, 70);
                ui->kickModeData.status.mode = 1;
            } else if (ui->kickModeData.status.mode == 1) {
                ui->display.setParttImage(120, 60, straightWhiteImg);
                ui->display.publish(180, 70);
                ui->kickModeData.status.mode = 0;
            }

        } else if (ui->touch.point.x > 180 && ui->touch.point.x < 300 && ui->touch.point.y > 150 && ui->touch.point.y < 210 && ui->kickModeData.status.charge == 1) {
            // chip
            if (ui->kickModeData.status.mode == 0) {
                ui->display.setParttImage(120, 60, chipRedImg);
                ui->display.publish(180, 150);
                ui->kickModeData.status.mode = 2;
            } else if (ui->kickModeData.status.mode == 2) {
                ui->display.setParttImage(120, 60, chipWhiteImg);
                ui->display.publish(180, 150);
                ui->kickModeData.status.mode = 0;
            }

        } else if (ui->touch.point.x > 20 && ui->touch.point.x < 150 && ui->touch.point.y > 90 && ui->touch.point.y < 210) {
            // _ui->modeData.status.mode = 0;
            // _ui->changeFlag_overMode = true;

            if (ui->kickModeData.status.charge == 0) {
                ui->display.setParttImage(130, 120, chargeImg);
                ui->display.publish(20, 90);
                ui->kickModeData.status.charge = 1;
            } else if (ui->kickModeData.status.charge == 1) {
                kickUI();
                ui->kickModeData.status.charge = 0;
            }
        }
    }
}

void KickTestMode::kickUI() {
    ui->display.createSprite(320, 240);
    ui->display.setBackgroundImage(kickImg);
    ui->display.publish();
}