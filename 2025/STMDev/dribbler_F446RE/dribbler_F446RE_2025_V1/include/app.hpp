#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "CAN.hpp"
#include "Definishion_Control.hpp"
#include "Basic_IO_Control.hpp"
#include "AD_Setup_Control.hpp"

// double map(double target,double min1,double max1,double min2,double max2){
//     return ((max2 - min2 + 1) / (max1 - min1 + 1)) * target;
// }

#ifdef __cplusplus
extern "C" {
#endif

void Setup(void);
void MainLoop(void);

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