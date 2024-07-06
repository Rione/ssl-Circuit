#ifndef __Media_Executor__
#define __Media_Executor__

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_POWER_PIN 11 // NeoPixelの電源
#define NEOPIXEL_PIN 12       // NeoPixel　の出力ピン番号
#define LED_COUNT 1           // LEDの連結数
#define WAIT_MS 1000          // 次の点灯までのウエイト
#define BRIGHTNESS 255        // 輝度
#define BUZZER_PIN 7          // ブザーのピン番号

enum playType {
    NONE,
    NOTIFY,
    START,
    STOP,
    ERROR,
    SUCCESS,
    WARNING,
    INFO,
};

class MediaExecutor {
  public:
    MediaExecutor();
    ~MediaExecutor();
    void init();
    void setBuzzerType(playType type) {
        BuzzerType = type;
    }
    playType getButtonType();
    void playFuncBuzzer(playType type);
    void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b);
    void execute();

    Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

  private:
    playType BuzzerPlayTypePrev = NONE;
    playType BuzzerType;
};

extern MediaExecutor media;

#endif