#include "app.hpp"
#include "AnalogIn.hpp"

void Setup(void){

}

void MainLoop(){
  while(1){
    Set_LED(RED,LOW);
    Set_LED(BLUE,HIGH);
    HAL_Delay(1000);
    Set_LED(RED,HIGH);
    Set_LED(BLUE,LOW);
    HAL_Delay(1000);
  }
}

void Set_LED(uint8_t coller,uint8_t status){
  switch(coller){
    case RED:
      if(status == HIGH){
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      }
    break;
    case BLUE:
      if(status == HIGH){
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
      }
    break;
    default: 
    break;
  }
}

void Motor_Rotate_Control(uint8_t mode,uint16_t speed){
  switch(mode){
    case FORWARD:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, map(speed,0,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_TIM3_FRQ_MIN);
    break;
    case REVERSE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, map(speed,0,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
    break;
    case BRAKE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_TIM3_FRQ_MAX);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_TIM3_FRQ_MAX);
    break;
    case FET_DISABLE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_TIM3_FRQ_MIN);
    break;
    default:
    break;
  }
}