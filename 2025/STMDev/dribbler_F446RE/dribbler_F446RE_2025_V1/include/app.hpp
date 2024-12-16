#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "MyMath.hpp"

#define RED 0
#define BLUE 1

#define FORWARD 0
#define REVERSE 1
#define BRAKE 2
#define FET_DISABLE 3

#define HIGH 0
#define LOW 1 

#define DRV_ENABLE HAL_GPIO_WritePin(MD_nSEEP_GPIO_Port, MD_nSEEP_Pin, GPIO_PIN_SET)
#define DRV_DISABLE HAL_GPIO_WritePin(MD_nSEEP_GPIO_Port, MD_nSEEP_Pin, GPIO_PIN_RESET)

#define PWM_TIM3_FRQ_MAX 799
#define PWM_TIM3_FRQ_MIN 0

double map(double target,double max1,double min1,double max2,double min2){
    return ((min1 - max1 + 1) / (min2 - max2 + 1)) * target;
}

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);
void Set_LED(uint8_t,uint8_t);
void Motor_Rotate_Control(uint8_t,uint16_t);

#ifdef __cplusplus
}
#endif

#endif