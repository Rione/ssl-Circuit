#include "home.hpp"

void Home::displaySet() {
    if (ui->changeFlag_overMode) {
        Serial.println("Home displaySet");
        homeUI();
        ui->changeFlag_overMode = false;
    }
}

void Home::determine() {
}

void Home::homeUI() {
    ui->display.setParttImage(0, 30, 320, 210, homeImg);
}