//basic_io_control

#ifndef __BIOC_HPP__
#define __BIOC_HPP__

#include "main.h"
#include "app.hpp"
#include "Definishion_Control.hpp"

#ifdef __cplusplus

extern "C" {
  class Basic_IO_Control_Extension_Sensor{
    public:
      void Ball_Sensor_Activate();
      void Ball_Sensor_Inactivate();
      void ENC_Activate();
      void ENC_Inactivate();
  };

  class Basic_IO_Control_LED{
    public:
      void RED(int status);
      void YELLOW(int status);
      void BLUE(int status);
      void GREEN(int status);
      void CAN_LED(int status);
      void BS_LED(int status);
      void ALL_Control(int status);
      void ALL_Control_EX_CAN(int status);
  };

  class Basic_IO_Control_LED_Flash{
    public:
      bool LED_Flash_Activate = false;
      uint8_t LED_Flash_RED = HOLD;
      uint8_t LED_Flash_YELLOW = HOLD;
      uint8_t LED_Flash_BLUE = HOLD;
      uint8_t LED_Flash_GREEN = HOLD;
      uint8_t LED_Flash_CAN = HOLD;
      void LED_Flash_Control();
  };

  class Basic_IO_Control_Motor{
    public:      
      void Reverse(int speed);
      void Forward(int speed);
      void Brake();
      void FET_DISABLE();
      void ENABLE();
      void DISABLE();
  };
}

#endif

#endif
