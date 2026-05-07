#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdio.h>

#include "adc.h"
#include "bldc.h"
#include "digitalinout.h"
#include "pwm_out.h"
#include "stspin32g4.h"
#include "tim.h"
#include "timer.h"

void Setup();
void MainApp();

#endif  // APP_H_