#include "basic_io_control.hpp"

void Motor_Control::Reverse(int speed){
  speed =  ((PWM_TIM3_FRQ_MAX - PWM_TIM3_FRQ_MIN + 1) / (100 - 1 + 1)) * speed;
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

void Motor_Control::Forward(int speed){
  speed =  ((PWM_TIM3_FRQ_MAX - PWM_TIM3_FRQ_MIN + 1) / (100 - 1 + 1)) * speed;
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
}

void Motor_Control::Brake(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MAX);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MAX);
}

void Motor_Control::FET_DISABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
}

void Motor_Control::ENABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
}

void Motor_Control::DISABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_RESET);
}



