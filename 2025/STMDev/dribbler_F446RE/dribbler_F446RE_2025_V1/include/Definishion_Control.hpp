//definishion_Control

#include "main.h"
#include "app.hpp"
#include "adc.h"

#ifndef __DC_HPP__
#define __DC_HPP__

#define START       1
#define STOP        2
#define HOLD        3

#define HIGH    0
#define LOW     1 

#define PWM_TIM3_FRQ_MAX 800
#define PWM_TIM3_FRQ_MIN 1

#define MOTOR_CURRENT   0
#define BALL_SENSOR_VAL 1
#define ENC1_VAL        2
#define ENC2_VAL        3

#define ADC_CONTINUE_NUM 5

#define DRV_MIN_CURRENT 47
#define DRV_MIN_CURRENT_MINUS_RANGE 20

#ifdef __cplusplus

extern "C" {
  const bool Fnc_Administrator_Privilege = true;
}

#endif

#endif
