#ifndef __APP__
#define __APP__
#ifdef __cplusplus

#include "main.h"

#include "adc.h"
#include <cstdio>
#include <cstring>

extern "C" {
#endif

void TimInterrupt500hz();
void chipKick(float power);
void straightKick(float power);
void setup();
void main_app();

#ifdef __cplusplus
}
#endif
#endif