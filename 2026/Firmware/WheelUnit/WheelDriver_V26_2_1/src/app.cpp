#include "app.hpp"

void Setup(void){
  HAL_GPIO_WritePin(LED_B3_GPIO_Port, LED_B3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_B2_GPIO_Port, LED_B2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_B1_GPIO_Port, LED_B1_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
}

void MainLoop(){
  // int time_ms = 5;
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  // HAL_Delay(time_ms);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  // HAL_Delay(time_ms);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  // HAL_Delay(time_ms);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 500);
  // HAL_Delay(time_ms);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 500);
  // HAL_Delay(time_ms);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 500);
  // HAL_Delay(time_ms);
}
