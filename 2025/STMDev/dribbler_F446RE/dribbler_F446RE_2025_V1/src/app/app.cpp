#include "app.hpp"
#include "adc.h"
#include "config.h"
//#include "CAN.hpp"

uint16_t adc_val_ch2[4];

uint16_t motor_max_speed = 0;
uint16_t motor_min_speed = 0;

// CANBus::CANData canRecvData;

// CANBus can = CANBus(&hcan1, 0);

void Setup(void){
  //ADC initialization
  HAL_ADC_Start(&hadc2);
  HAL_Delay(50);  //Do not delete this delay!!
  HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,4);
  for(uint8_t i = 0;i < 4;i++){
    while(!(adc_val_ch2[i] > 0));
  }


    int p = 0;
    printf("%d\n",p);

  //Motor initialization
  // Set_LED(USER_LED_GREEN,HIGH);
  // Set_LED(USER_LED_BLUE,HIGH);
  // Set_LED(USER_LED_RED,HIGH);
  // Set_LED(USER_LED_YELLOW,HIGH);

  HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(test2_GPIO_Port, test2_Pin, GPIO_PIN_SET);
  
}

void MainLoop(){
  while(1){
    Set_LED(USER_LED_YELLOW,LOW);
    // uint16_t thr = 750;
    // CANBus::CANData data = {
    //     .stdId = 0x09,
    //     .data = {
    //         (uint8_t)(thr & 0xFF),
    //         (uint8_t)((thr >> 8) & 0xFF),
    //     },
    // };
    // can.send(data);
    int p = 0;
    //printf("aa\n");

    HAL_Delay(100);

    Set_LED(USER_LED_YELLOW,HIGH);
    HAL_Delay(100);
  }
}

void Set_LED(int coller,int status){
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

void Motor_Rotate(int mode,int speed){
  int correction_speed = map(speed,motor_min_speed,motor_max_speed,PWM_TIM3_FRQ_MIN,PWM_TIM3_FRQ_MAX);
  switch(mode){
    case FORWARD:
      DRV_Control(MD_ENABLE);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, correction_speed);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    break;
    case REVERSE:
      DRV_Control(MD_ENABLE);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, correction_speed);
    break;
    case FET_DISABLE:
      DRV_Control(MD_ENABLE);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    break;
    default:
    break;
  }
}

void Motor_Brake(){
  DRV_Control(MD_ENABLE);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MAX);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MAX);
}

void DRV_Control(int mode){
  switch(mode){
    case FET_DISABLE:
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    break;
    case MD_ENABLE:
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
    break;
    case MD_DISABLE:
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_RESET);
    break;
    default:
    break;
  }
}
