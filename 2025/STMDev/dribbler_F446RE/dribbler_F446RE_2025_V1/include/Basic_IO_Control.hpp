//basic_io_control

#include "main.h"
#include "app.hpp"

class Basic_IO_Control_Extension_Sensor{
  public:
    void Ball_Sensor_Activate();
    void Ball_Sensor_Inactivate();
    void ENC_Activate();
    void ENC_Inactivate();
};

class Basic_IO_Control_LED{
  public:
    bool LED_Flash_Activate = false;
    uint8_t LED_Flash_RED_100ms = HOLD;
    uint8_t LED_Flash_YELLOW_100ms = HOLD;
    uint8_t LED_Flash_BLUE_100ms = HOLD;
    uint8_t LED_Flash_GREEN_100ms = HOLD;
    uint8_t LED_Flash_CAN_100ms = HOLD;

    void RED(int status);
    void YELLOW(int status);
    void BLUE(int status);
    void GREEN(int status);
    void CAN_LED(int status);
    void BS_LED(int status);
    void ALL_Control(int status);
    void ALL_Control_EX_CAN(int status);
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