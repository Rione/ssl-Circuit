//basic_io_control_LED

#include "main.h"
#include "app.hpp"

class LED_Control {
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