#ifndef __APP__
#define __APP__
#ifdef __cplusplus

#include "main.h"

extern "C" {
#endif
void setup();
void main_app();
void processData(int data);
void chargeDevice();
int detectphoto(int adc_value);
void straightkick();
void chipkick();
void dribble();
void deactivateAll();
int readADC();
void updatePhotoDetection(int adcValue);
void dribblestop();
// void select();
#ifdef __cplusplus
}
#endif
#endif