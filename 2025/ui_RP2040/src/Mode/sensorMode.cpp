#include "sensorMode.hpp"

void SensorMode::displaySet() {
    if (ui->changeFlag_overMode) {
        drawUI();
        updateBallSensor(true);
        updateMotors(true);
    } else {
        updateBallSensor(false);
        updateMotors(false);
    }
}

void SensorMode::determine() {
    // Sensor screen doesn't have interactive buttons other than sidebar (handled in ui_kit)
}

void SensorMode::drawUI() {
    // Clear content area
    ui->display.sprite->fillRect(72, 28, 248, 212, ui->colBg);

    // Ball Sensor Card
    ui->display.sprite->drawRect(76, 32, 240, 44, ui->colBorder);
    ui->display.sprite->setTextColor(ui->colTextMuted);
    ui->display.sprite->setTextDatum(TL_DATUM);
    ui->display.sprite->drawString("BALL SENSOR", 82, 36);

    // Motors Card
    ui->display.sprite->drawRect(76, 80, 240, 156, ui->colBorder);
    ui->display.sprite->setTextColor(ui->colTextMuted);
    ui->display.sprite->setTextDatum(TL_DATUM);
    ui->display.sprite->drawString("MOTORS (4-WHEEL)", 82, 84);

    ui->display.publish(72, 28, 248, 212);
}

void SensorMode::updateBallSensor(bool force) {
    if (ui->info.ballSensor != prevBallSensor || force) {
        prevBallSensor = ui->info.ballSensor;

        ui->display.sprite->fillRect(80, 50, 232, 22, ui->colBg); // clear inner area
        
        if (ui->info.ballSensor) {
            ui->display.sprite->fillCircle(88, 61, 5, ui->colSuccess);
            ui->display.sprite->setTextColor(ui->colFg);
            ui->display.sprite->setTextDatum(ML_DATUM);
            ui->display.sprite->drawString("Ball", 100, 61);

            ui->display.sprite->drawRect(160, 50, 70, 20, ui->colSuccess);
            ui->display.sprite->setTextColor(ui->colSuccess);
            ui->display.sprite->setTextDatum(MC_DATUM);
            ui->display.sprite->drawString("DETECTED", 195, 61);
        } else {
            ui->display.sprite->drawCircle(88, 61, 5, ui->colTextMuted);
            ui->display.sprite->setTextColor(ui->colFg);
            ui->display.sprite->setTextDatum(ML_DATUM);
            ui->display.sprite->drawString("Ball", 100, 61);

            ui->display.sprite->drawRect(160, 50, 70, 20, ui->colTextMuted);
            ui->display.sprite->setTextColor(ui->colTextMuted);
            ui->display.sprite->setTextDatum(MC_DATUM);
            ui->display.sprite->drawString("NONE", 195, 61);
        }

        ui->display.publish(80, 50, 232, 22);
    }
}

void SensorMode::updateMotors(bool force) {
    static float prevVel[4] = {0};
    uint8_t currStatus = 0;
    for(int i=0; i<4; i++) {
        if(ui->info.motor[i].statusOk) currStatus |= (1 << i);
    }

    bool updated = force || (currStatus != prevMotorStatus);
    for(int i=0; i<4; i++) {
        if(ui->info.motor[i].velocity != prevVel[i]) updated = true;
    }

    if (updated) {
        prevMotorStatus = currStatus;
        
        for (int i=0; i<4; i++) {
            prevVel[i] = ui->info.motor[i].velocity;
            
            int mx = 80 + i * 58;
            int my = 100;
            int mw = 54;
            int mh = 130;

            ui->display.sprite->fillRect(mx, my, mw, mh, ui->colMuted);
            ui->display.sprite->drawRect(mx, my, mw, mh, ui->colBorder);

            ui->display.sprite->setTextColor(ui->colFg);
            ui->display.sprite->setTextDatum(MC_DATUM);
            ui->display.sprite->drawString("M" + String(i+1), mx + mw/2, my + 15);

            bool ok = ui->info.motor[i].statusOk;
            ui->display.sprite->setTextColor(ok ? ui->colSuccess : ui->colError);
            ui->display.sprite->drawString(ok ? "OK" : "ERR", mx + mw/2, my + 45);

            ui->display.sprite->setTextColor(ui->colTextMuted);
            ui->display.sprite->drawString(String(ui->info.motor[i].velocity, 1), mx + mw/2, my + 80);
            ui->display.sprite->drawString("rad/s", mx + mw/2, my + 100);
        }

        ui->display.publish(80, 100, 232, 130);
    }
}
