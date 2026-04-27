#ifndef MOTOR_H_
#define MOTOR_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "maf.h"
#include "pwm_out.h"

#define MAX_SPEED_LEVEL 5
#define BASE_CURRENT_MEASURE_NUM 500
#define MOTOR_CURRENT_THRESHOLD 50

void Motor_Init();
void Motor_Update(uint16_t current_val);
bool Motor_SetBaseCurrent(uint16_t current_val);
void Motor_Drive(uint8_t level);
void Motor_Brake();
void Motor_Free();

bool Motor_IsBallCaptured();

#endif // MOTOR_H_