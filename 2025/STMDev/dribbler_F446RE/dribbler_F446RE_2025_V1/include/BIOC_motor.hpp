//basic_io_control_LED

#include "main.h"
#include "app.hpp"

class Motor_Control {
  public:      
    void Reverse(int speed);
    void Forward(int speed);
    void Brake();
    void FET_DISABLE();
    void ENABLE();
    void DISABLE();
};