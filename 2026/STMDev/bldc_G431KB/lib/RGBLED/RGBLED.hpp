#ifndef __RGBLED__
#define __RGBLED__
#ifdef __cplusplus
#include "main.h"
#include "DigitalInOut.hpp"

typedef enum {
    RED,
    GREEN,
    BLUE,
    CYAN,
    MAGENTA,
    YELLOW,
    WHITE,
    OFF
} rgb_t;

class RGBLED {
  public:
    RGBLED() : led_red(LED_R_GPIO_Port, LED_R_Pin),
               led_green(LED_G_GPIO_Port, LED_G_Pin),
               led_blue(LED_B_GPIO_Port, LED_B_Pin) {
        led(rgb_t::OFF);
    }
    ~RGBLED() {
    }
    void led(rgb_t color) {
        switch (color) {
        case rgb_t::RED:
            led_red = 0;
            led_green = 1;
            led_blue = 1;
            break;
        case rgb_t::GREEN:
            led_red = 1;
            led_green = 0;
            led_blue = 1;
            break;
        case rgb_t::BLUE:
            led_red = 1;
            led_green = 1;
            led_blue = 0;
            break;
        case rgb_t::YELLOW:
            led_red = 0;
            led_green = 0;
            led_blue = 1;
            break;
        case rgb_t::CYAN:
            led_red = 1;
            led_green = 0;
            led_blue = 0;
            break;
        case rgb_t::MAGENTA:
            led_red = 0;
            led_green = 1;
            led_blue = 0;
            break;
        case rgb_t::WHITE:
            led_red = 0;
            led_green = 0;
            led_blue = 0;
            break;
        case rgb_t::OFF:
        default:
            led_red = 1;
            led_green = 1;
            led_blue = 1;
        }
    }

  private:
    DigitalOut led_red;
    DigitalOut led_green;
    DigitalOut led_blue;
};

#endif
#endif