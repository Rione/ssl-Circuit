#include "app.hpp"
#include "adc.h"

uint16_t adc_val_ch2[4];

void Setup(void){
  HAL_ADC_Start(&hadc2);
  HAL_Delay(50);  //Do not delete this delay!!
  HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,4);
  for(uint8_t i = 0;i < 4;i++){
    while(!(adc_val_ch2[i] > 0));
  }
  // Set_LED(USER_LED_BLUE,HIGH);
  // Set_LED(USER_LED_GREEN,HIGH);
  // Set_LED(USER_LED_YELLOW,HIGH);
  // Set_LED(USER_LED_RED,HIGH);
}

void MainLoop(){
  while(1){
    DRV_ENABLE;
    Motor_Rotate_Control(FORWARD,70);
    HAL_Delay(300);
    Motor_Rotate_Control(BRAKE,0);
    HAL_Delay(100);
    Motor_Rotate_Control(REVERSE,70);
    HAL_Delay(300);
    Motor_Rotate_Control(BRAKE,0);
    HAL_Delay(100);
    //Motor_Rotate_Control(REVERSE,50);
    //HAL_Delay(50);

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
