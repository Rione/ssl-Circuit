#ifndef _RIMODES_
#define _RIMODES_
#include "mbed.h"

typedef struct {
    char *modeName;
    char modeLetter;
    mbed::Callback<void()> before;
    mbed::Callback<void()> body;
    mbed::Callback<void()> after;
} RIMode;

#endif