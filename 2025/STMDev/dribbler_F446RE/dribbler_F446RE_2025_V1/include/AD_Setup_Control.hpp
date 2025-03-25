//AD_Setup_Control

#ifndef __ADSC_HPP__
#define __ADSC_HPP__

#include "math.h"
#include "main.h"
#include "app.hpp"
#include "adc.h"
#include "Definishion_Control.hpp"

#ifdef __cplusplus

extern "C" {
  extern uint16_t sw_val;
  extern uint16_t adc_val_ch1[4];

  class AD_Setup_Control{
    public:
      void Administrator_Privilege();
      void ADC_Check();
      void DRV_Check();
      void Motor_Check();
  };
}

#endif

#endif

