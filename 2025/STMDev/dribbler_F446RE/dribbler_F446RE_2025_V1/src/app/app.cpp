#include "app.hpp"
#include "adc.h"

uint16_t adc_val[3];

void Setup(void){
  read_ADC();
}

void MainLoop(){
  while(1){
    Set_LED(BLUE,HIGH);
    Set_LED(RED,HIGH);
    Motor_Rotate_Control(FORWARD,100);

    int t = 0;
    for(int i = 0;i < 50;i++){
      HAL_Delay(1);
      t += adc_val[1];
    }
    //HAL_Delay(50);
    printf("%d\n",t / 50);

    //DRV_ENABLE;

    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1);
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
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, map(speed,1,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_TIM3_FRQ_MIN);
    break;
    case REVERSE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, map(speed,1,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
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

void read_ADC(){
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val,3);
  for(uint8_t i = 0;i < 3;i++){
    while(!(adc_val[i] > 0));
  }
}