#include "MediaExecutor.hpp"

static unsigned long startTime = 0;

MediaExecutor::MediaExecutor() {
}

MediaExecutor::~MediaExecutor() {
}

void MediaExecutor::init() {
    // Neopixelの電源供給開始
    pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_POWER_PIN, HIGH);

    // ブザーピンの明示的な初期化を追加
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    pixels.begin(); // NeoPixel制御開始
}

playType MediaExecutor::getButtonType() {
    return BuzzerType;
}

void MediaExecutor::playFuncBuzzer(playType type) {
    if (BuzzerPlayTypePrev != type) {
        startTime = millis();
        BuzzerPlayTypePrev = type;
        BuzzerType = type;
    }
}

void MediaExecutor::setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

void MediaExecutor::execute() {
    switch (BuzzerType) {
        case NONE:
            noTone(BUZZER_PIN);
            break;
        case NOTIFY:
            if (millis() - startTime < 50) tone(BUZZER_PIN, 10000);
            else if (millis() - startTime < 110) tone(BUZZER_PIN, 4200);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case START:
            if (millis() - startTime < 20) tone(BUZZER_PIN, 3900);
            else if (millis() - startTime < 70) tone(BUZZER_PIN, 3000);
            else if (millis() - startTime < 100) tone(BUZZER_PIN, 6000);
            else if (millis() - startTime < 120) tone(BUZZER_PIN, 10000);
            else if (millis() - startTime < 170) tone(BUZZER_PIN, 4200);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case STOP:
            if (millis() - startTime < 100) tone(BUZZER_PIN, 9000);
            else if (millis() - startTime < 200) tone(BUZZER_PIN, 7200);
            else if (millis() - startTime < 300) tone(BUZZER_PIN, 1200);
            else if (millis() - startTime < 400) tone(BUZZER_PIN, 8000);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case ERROR:
            if (millis() - startTime < 50) tone(BUZZER_PIN, 2200);
            else if (millis() - startTime < 100) tone(BUZZER_PIN, 3200);
            else if (millis() - startTime < 150) tone(BUZZER_PIN, 600);
            else if (millis() - startTime < 250) tone(BUZZER_PIN, 1200);
            else if (millis() - startTime < 300) tone(BUZZER_PIN, 700);
            else if (millis() - startTime < 350) tone(BUZZER_PIN, 500);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case SUCCESS:
            if (millis() - startTime < 20) tone(BUZZER_PIN, 8000);
            else if (millis() - startTime < 70) tone(BUZZER_PIN, 11000);
            else if (millis() - startTime < 120) tone(BUZZER_PIN, 4000);
            else if (millis() - startTime < 150) tone(BUZZER_PIN, 8000);
            else if (millis() - startTime < 200) tone(BUZZER_PIN, 6000);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case WARNING:
            if (millis() - startTime < 100) tone(BUZZER_PIN, 7000);
            else if (millis() - startTime < 200) tone(BUZZER_PIN, 2500);
            else if (millis() - startTime < 300) tone(BUZZER_PIN, 1800);
            else if (millis() - startTime < 400) tone(BUZZER_PIN, 1500);
            else if (millis() - startTime < 500) tone(BUZZER_PIN, 800);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
        case INFO:
            if (millis() - startTime < 400) tone(BUZZER_PIN, 1500);
            else { noTone(BUZZER_PIN); BuzzerType = NONE; }
            break;
    }
}
