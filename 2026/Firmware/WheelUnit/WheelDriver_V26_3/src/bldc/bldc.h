#ifndef BLDC_H_
#define BLDC_H_

#include <stdio.h>

#include "main.h"
#include "mymath.h"
#include "pwm_out.h"

#define MIN_DUTY 0.3
#define MAX_DUTY 0.7

void BLDC_Init();
void BLDC_OpenLoopDrive(double amp, double phase);

#endif  // BLDC_H_