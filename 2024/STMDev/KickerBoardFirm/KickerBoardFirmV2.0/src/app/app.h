#ifndef __APP__
#define __APP__
#ifdef __cplusplus

#include "main.h"

#include <cstdio>
#include <cstring>

extern "C" {
#endif

void TimInterrupt500hz();
void chipKick();
void straightKick();
void setup();
void main_app();

#ifdef __cplusplus
}
#endif
#endif