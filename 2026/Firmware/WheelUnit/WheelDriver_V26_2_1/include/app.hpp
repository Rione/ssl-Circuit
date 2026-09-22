#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
//#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
void Setup(void);
void MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif
