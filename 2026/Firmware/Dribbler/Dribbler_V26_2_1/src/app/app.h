#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdio.h>

#include "adc.h"
#include "can_bus.h"
#include "digitalinout.h"
#include "lpf.h"
#include "main.h"
#include "dribbler.h"
#include "motor.h"
#include "pwm_out.h"
#include "serial.h"
#include "timer.h"

void Setup();
void MainApp();

#endif  // APP_H_