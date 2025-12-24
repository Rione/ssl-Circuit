#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdio.h>

#include "adc.h"
#include "can.h"
#include "digitalinout.h"
#include "lpf.h"
#include "main.h"
#include "pwm_out.h"
// #include "serial.h"
#include "motor.h"
#include "timer.h"

void Setup();
void MainApp();

#endif  // APP_H_