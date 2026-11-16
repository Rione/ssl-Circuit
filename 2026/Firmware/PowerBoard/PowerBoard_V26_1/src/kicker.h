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
bool Kicker_Kick(int kickType, float power);

// ダイレクトキック(MainBoardを介さず、Dribblerのフォトセンサ反応で即キック)
// type: 0=解除, 1=ストレート, 2=チップ / power: 0-255(MainBoardで換算済みの値)
void Kicker_SetDirectKick(uint8_t type, uint8_t power);
void Kicker_OnPhotoSensor(bool detected);
void Kicker_Update();
void Kicker_RequestCharge();
void Kicker_RequestDischarge();

bool Kicker_DoneCheck();

#endif  // KICKER_H_