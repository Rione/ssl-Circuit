#ifndef DRIBBLER_H_
#define DRIBBLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "digitalinout.h"
#include "lpf.h"
#include "motor.h"

void Dribbler_Init();
void Dribbler_Update(uint16_t photo_val, uint16_t current_val);

bool Dribbler_SetPhotoThreshold(uint16_t photo_val);

// フォトセンサのみによる判定
bool Dribbler_IsBallCapturedByPhoto();

// 電流のみによる判定
bool Dribbler_IsBallCapturedByCurrent();

// 総合判定
bool Dribbler_IsBallCaptured();

#endif  // DRIBBLER_H_
