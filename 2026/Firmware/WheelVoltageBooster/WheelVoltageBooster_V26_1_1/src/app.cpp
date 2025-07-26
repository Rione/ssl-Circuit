#include "app.hpp"

void Setup(void){
  
}

void MainLoop(){
  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
}

