#include "app.hpp"

void Setup(void){
  
}

void MainLoop(){
  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_CAN_GPIO_Port,LED_CAN_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_CAN_GPIO_Port,LED_CAN_Pin,GPIO_PIN_RESET);
  HAL_Delay(1000);

  printf("Hello World!\n");
}

