#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "CAN.hpp"

#define HIGH    0
#define LOW     1 

#define PWM_TIM3_FRQ_MAX 800
#define PWM_TIM3_FRQ_MIN 1

double map(double target,double min1,double max1,double min2,double max2){
    return ((max2 - min2 + 1) / (max1 - min1 + 1)) * target;
}

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);

void ADC_setup();

#ifdef __cplusplus
}
#endif

#endif