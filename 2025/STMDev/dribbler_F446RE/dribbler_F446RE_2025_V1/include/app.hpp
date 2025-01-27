#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"

#define USER_LED_RED        1
#define USER_LED_YELLOW     2
#define USER_LED_BLUE       3
#define USER_LED_GREEN      4
#define CAN_LED             5

#define FORWARD         0
#define REVERSE         1
#define BRAKE           2
#define FET_DISABLE     3

#define HIGH    0
#define LOW     1 

#define DRV_ENABLE HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET)
#define DRV_DISABLE HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_RESET)

#define PWM_TIM3_FRQ_MAX 1000
#define PWM_TIM3_FRQ_MIN 1

double map(double target,double min1,double max1,double min2,double max2){
    return ((max2 - min2 + 1) / (max1 - min1 + 1)) * target;
}

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);
void Set_LED(uint8_t,uint8_t);
void Motor_Rotate_Control(uint8_t,uint16_t);
void read_ADC();

#ifdef __cplusplus
}
#endif

#endif