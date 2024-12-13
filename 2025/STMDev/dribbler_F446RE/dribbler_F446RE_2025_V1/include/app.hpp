#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "MyMath.hpp"

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