#include "display.h"

DISPLAY_DEVICE::DISPLAY_DEVICE(TFT_eSPI *tftPtr, TFT_eSprite *sprite) {
    pinMode(backlightPin, OUTPUT);
    // digitalWrite(backlightPin, HIGH);

    digitalWrite(backlightPin, LOW);

    this->tftPtr = tftPtr;
    this->sprite = sprite;
}

void DISPLAY_DEVICE::init(void) {
    sprite->createSprite(320, 240);
    sprite->setColorDepth(8);

    tftPtr->begin();
    tftPtr->setRotation(1);

    pinMode(TFT_CS, OUTPUT);

    digitalWrite(backlightPin, HIGH);

}

void DISPLAY_DEVICE::setSPIClockFast(void) {
    SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));
}

void DISPLAY_DEVICE::setBackgroundImage(const uint16_t *imagePtr) {
    sprite->pushImage(0, 0, 320, 240, imagePtr);
    publish(0, 0);
}

void DISPLAY_DEVICE::setParttImage(int x, int y, int w, int h, const uint16_t *imagePtr) {
    // sprite->createSprite(w, h);
    sprite->pushImage(x, y, w, h, imagePtr);
    publish(0, 0);
}

void DISPLAY_DEVICE::publish(int x, int y) {
    setSPIClockFast();
    sprite->pushSprite(x, y);
    // sprite->deleteSprite();
}

void DISPLAY_DEVICE::createSprite(int x, int y) {
    sprite->createSprite(x, y);
    sprite->setColorDepth(8);
}

void DISPLAY_DEVICE::setMainImage(const uint16_t *imagePtr) {
    // info tabを抜いた320x210の画像をセットする
    sprite->pushImage(0, 30, 320, 210, imagePtr);
    publish(0, 0);
}


