#pragma once
#include "stm32f4xx_hal.h"
#include <cstdio>

#define delay(ms) HAL_Delay(ms)

struct SerialMock {
    void print(const char* s) { printf("%s", s); }
    void println(const char* s) { printf("%s\r\n", s); }
    void print(float v, int d) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", d, v);
        printf("%s", buf);
    }
};

extern SerialMock Serial;

