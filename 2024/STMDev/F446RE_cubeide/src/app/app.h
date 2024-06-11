#ifndef __APP__
#define __APP__
#ifdef __cplusplus

#include "main.h"

extern "C" {
#endif
void setup();
void main_app();

void TimInterrupt1khz();
void TimInterrupt4khz();
void canRxInterrupt(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif
#endif