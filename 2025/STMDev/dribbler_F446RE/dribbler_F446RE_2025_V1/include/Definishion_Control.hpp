//definishion_Control

#include "main.h"
#include "app.hpp"
#include "adc.h"

#ifndef __DC_HPP__
#define __DC_HPP__

//System Definishion BEGIN ** Do Not Change **

#define START       1
#define STOP        2
#define HOLD        3

#define HIGH    0
#define LOW     1 

#define MOTOR_CURRENT   0
#define BALL_SENSOR_VAL 1
#define ENC1_VAL        2
#define ENC2_VAL        3

//System Definishion END

#define PWM_TIM3_FRQ_MAX 800
#define PWM_TIM3_FRQ_MIN 1

#define Motor_Base_Current 100
#define Motor_Base_Current_RANGE 10
#define Motor_Current_Differ_Tolerance 20

#define Main_Power_Constant 10
#define Main_Power_Constant_Range 100

#define PHOTO_THRESHOLD 100
#define MOTOR_CURRENT_THRESHOLD 113

#ifdef __cplusplus

extern "C" {
  const bool Fnc_Administrator_Privilege = true;
}

#endif

#endif
