#include "app.hpp"
#include "adc.h"

uint16_t adc_val_ch1[1];
uint16_t adc_val_ch2[2];
uint16_t adc_val_ch3[1];

void Setup(void){
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,1);
  for(uint8_t i = 0;i < 1;i++){
    while(!(adc_val_ch1[i] > 0));
  }
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,2);
  for(uint8_t i = 0;i < 2;i++){
    while(!(adc_val_ch2[i] > 0));
  }
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t *)&adc_val_ch3,1);
  for(uint8_t i = 0;i < 1;i++){
    while(!(adc_val_ch3[i] > 0));
  }
}

void MainLoop(){
  while(1){
    
  }
}

void Set_LED(uint8_t coller,uint8_t status){
  switch(coller){
    case USER_LED_RED:
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
      }
    break;
    case USER_LED_YELLOW:
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
      }
    break;
    case USER_LED_GREEN:
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
      }
    break;
    case USER_LED_BLUE:
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
      }
    break;
    case CAN_LED:
      if(status == HIGH){
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
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
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, map(speed,1,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    break;
    case REVERSE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, map(speed,1,100,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX));
    break;
    case BRAKE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MAX);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MAX);
    break;
    case FET_DISABLE:
      DRV_ENABLE;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    break;
    default:
    break;
  }
}
