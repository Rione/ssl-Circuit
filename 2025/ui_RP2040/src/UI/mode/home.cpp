#include "home.hpp"

void Home::displaySet(UiKit *_ui) {
    if (_ui->changeFlag_overMode) {
        Serial.println("Home displaySet");
        homeUI(_ui);
        _ui->changeFlag_overMode = false;
    }
}

void Home::determine(UiKit *_ui) {
}

void Home::homeUI(UiKit *_ui) {

    display.setParttImage(320, 210, homeImg);
    display.publish(0, 30);
}