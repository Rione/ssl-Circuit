#ifndef KICKER_H_
#define KICKER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "digitalinout.h"
#include "pwm_out.h"

void Kicker_Init();
void Kicker_Kick(int kickType, float power);
void Kicker_Charge();
void Kicker_Discharge();

bool Kicker_DoneCheck();

#endif  // KICKER_H_