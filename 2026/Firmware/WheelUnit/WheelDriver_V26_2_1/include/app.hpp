#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
//#include "tim.h"

#ifdef __cplusplus

extern "C" {
  extern TIM_HandleTypeDef htim1;
  void Setup(void);
  void MainLoop(void);
}

#endif

#endif
