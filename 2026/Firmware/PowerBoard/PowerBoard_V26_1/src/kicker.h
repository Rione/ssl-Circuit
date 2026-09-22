#ifndef KICKER_H_
#define KICKER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "digitalinout.h"
#include "pwm_out.h"
#include "timer.h"

void Kicker_Init();
void Kicker_SetBoostVoltage(float voltage);
void Kicker_Kick(int kickType, float power);
void Kicker_Update();
void Kicker_Charge();
void Kicker_Discharge();

bool Kicker_DoneCheck();

#endif  // KICKER_H_