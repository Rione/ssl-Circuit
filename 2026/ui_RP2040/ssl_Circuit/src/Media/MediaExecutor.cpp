#include "MediaExecutor.hpp"

MediaExecutor::MediaExecutor() {
}

MediaExecutor::~MediaExecutor() {
}

void MediaExecutor::init() {
    // Neopixelの電源供給開始
    pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_POWER_PIN, HIGH);

    pixels.begin(); // NeoPixel制御開始
}

playType MediaExecutor::getButtonType() {
    return BuzzerType;
}

void MediaExecutor::playFuncBuzzer(playType type) {
    if (type != BuzzerPlayTypePrev) {
        switch (type) {
        case NONE:
            noTone(BUZZER_PIN);
            break;
        case NOTIFY:
            tone(BUZZER_PIN, 10000);
            delay(50);
            tone(BUZZER_PIN, 4200);
            delay(60);
            noTone(BUZZER_PIN);
            break;
        case START:
            tone(BUZZER_PIN, 3900);
            delay(20);
            tone(BUZZER_PIN, 3000);
            delay(50);
            tone(BUZZER_PIN, 6000);
            delay(30);
            tone(BUZZER_PIN, 10000);
            delay(20);
            tone(BUZZER_PIN, 4200);
            delay(50);
            noTone(BUZZER_PIN);
            break;
        case STOP:
            tone(BUZZER_PIN, 9000);
            delay(100);
            tone(BUZZER_PIN, 7200);
            delay(100);
            tone(BUZZER_PIN, 1200);
            delay(100);
            tone(BUZZER_PIN, 8000);
            delay(100);
            noTone(BUZZER_PIN);
            break;
        case ERROR:
            tone(BUZZER_PIN, 2200);
            delay(50);
            tone(BUZZER_PIN, 3200);
            delay(50);
            tone(BUZZER_PIN, 600);
            delay(50);
            tone(BUZZER_PIN, 1200);
            delay(100);
            tone(BUZZER_PIN, 700);
            delay(50);
            tone(BUZZER_PIN, 500);
            delay(50);
            noTone(BUZZER_PIN);
            break;
        case SUCCESS:
            tone(BUZZER_PIN, 8000);
            delay(20);
            tone(BUZZER_PIN, 11000);
            delay(50);
            tone(BUZZER_PIN, 4000);
            delay(50);
            tone(BUZZER_PIN, 8000);
            delay(30);
            tone(BUZZER_PIN, 6000);
            delay(50);
            noTone(BUZZER_PIN);
            break;
        case WARNING:
            tone(BUZZER_PIN, 7000);
            delay(100);
            tone(BUZZER_PIN, 2500);
            delay(100);
            tone(BUZZER_PIN, 1800);
            delay(100);
            tone(BUZZER_PIN, 1500);
            delay(100);
            tone(BUZZER_PIN, 800);
            delay(100);
            noTone(BUZZER_PIN);
            break;
        case INFO:
            tone(BUZZER_PIN, 1500);
            delay(400);
            noTone(BUZZER_PIN);
            break;
        };
        BuzzerType = NONE;
    }
    BuzzerPlayTypePrev = type;
}

void MediaExecutor::setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

void MediaExecutor::execute() {
    playFuncBuzzer(BuzzerType);
}
