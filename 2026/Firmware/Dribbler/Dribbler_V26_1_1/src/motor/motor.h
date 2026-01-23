#ifndef MOTOR_H_
#define MOTOR_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mymath.h"
#include "pid.h"
#include "pwm_out.h"

void Motor_Init();
void Motor_Drive(float duty);
void Motor_Brake();

#endif  // MOTOR_H_