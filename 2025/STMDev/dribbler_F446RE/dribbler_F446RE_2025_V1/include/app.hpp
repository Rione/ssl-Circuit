#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "CAN.hpp"

#define HIGH    0
#define LOW     1 

#define PWM_TIM3_FRQ_MAX 800
#define PWM_TIM3_FRQ_MIN 1

#define MOTOR_CURRENT   0
#define ENC1_VAL        1
#define ENC2_VAL        2
#define BALL_SENSOR_VAL 3

#define DRV_MIN_CURRENT 47
#define DRV_MIN_CURRENT_MINUS_RANGE 10

double map(double target,double min1,double max1,double min2,double max2){
    return ((max2 - min2 + 1) / (max1 - min1 + 1)) * target;
}

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);

void ADC_Setup();
void DRV_Setup();

#ifdef __cplusplus
}
#endif

#endif