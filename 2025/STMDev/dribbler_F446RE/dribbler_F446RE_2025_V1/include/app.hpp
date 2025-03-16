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
#define BALL_SENSOR_VAL 1
#define ENC1_VAL        2
#define ENC2_VAL        3

#define ADC_CONTINUE_NUM 5

#define DRV_MIN_CURRENT 47
#define DRV_MIN_CURRENT_MINUS_RANGE 10

#define START       1
#define STOP        2
#define HOLD        3

// double map(double target,double min1,double max1,double min2,double max2){
//     return ((max2 - min2 + 1) / (max1 - min1 + 1)) * target;
// }

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);

void Set_Administrator_Privilege();
void Set_ADC();
void Set_ADC_Restart();
void Set_DRV();
void Set_Main_Motor();
void Interrupt_Processing_f10ms();
void Interrupt_Processing_f1ms();
void IPf10ms_LED_Flash_Control();
void HAL_CAN_Data_Output_ID0x1d2_466();
void HAL_CAN_Data_Input_ID0x1d1_465();

#ifdef __cplusplus
}
#endif

#endif