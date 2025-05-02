#ifndef __APP_HPP__
#define __APP_HPP__

#include "main.h"
#include "tim.h"
#include "CAN.hpp"
#include "Parameter_Control.hpp"
#include "IO_Basic_Control.hpp"
#include "IO_Setup_Control.hpp"

#ifdef __cplusplus

extern "C" {
  void Setup(void);
  void MainLoop(void);
  void Interrupt_Processing_f10ms();
  void Interrupt_Processing_f1ms();
  void CAN_Data_Output_ID0x1d2_246();
  void CAN_Data_Input_ID0x1d1_245();
}

#endif

#endif
