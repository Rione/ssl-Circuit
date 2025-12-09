#include "app.hpp"

void Setup(void){
  
}

void MainLoop(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}
