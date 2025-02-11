#include "app.hpp"
#include "adc.h"
#include "config.h"

uint16_t adc_val_ch2[4];

CANBus can = CANBus(&hcan1, 0);
CANBus::CANData canRecvData;

class Motor_Control {
  public:      
    void Reverse(int speed){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, map(speed, 1, 100, PWM_TIM3_FRQ_MIN, PWM_TIM3_FRQ_MAX));
    };

    void Forward(int speed){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, map(speed, 1, 100, PWM_TIM3_FRQ_MIN, PWM_TIM3_FRQ_MAX));
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    };

    void Brake(){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MAX);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MAX);
    };

    void FET_DISABLE(){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
    };

    void ENABLE(){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
    };

    void DISABLE(){
      HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_RESET);
    };
};

Motor_Control Main_motor;

void Setup(void){
  HAL_Delay(500);
  ADC_setup();
}

void MainLoop(){
  while(1){
    uint16_t thr = 750;
    CANBus::CANData data = {
        .stdId = 0x09,
        .data = {
            (uint8_t)(thr & 0xFF),
            (uint8_t)((thr >> 8) & 0xFF),
        },
    };
    can.send(data);

    printf("in\n");

    Set_LED(USER_LED_GREEN,HIGH);

    Main_motor.Forward(70);
    HAL_Delay(100);
    Main_motor.Brake();
    HAL_Delay(100);
    Main_motor.Reverse(70);
    HAL_Delay(100);
    Main_motor.Brake();
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
    case USER_LED_BLUE:
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
      }
    break;
    case USER_LED_GREEN:
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

void ADC_setup(){

  //ADC initialization

  printf("\n");

  printf("*** Start Initalization ***\n");
  Set_LED(USER_LED_RED,HIGH);
  HAL_Delay(1000);
  Set_LED(USER_LED_RED,LOW);

  printf("HAL_ADC_Start ---- ");
  Set_LED(USER_LED_BLUE,HIGH);
  HAL_ADC_Start(&hadc2);
  printf("Success!\n");
  HAL_Delay(1000);
  Set_LED(USER_LED_BLUE,LOW);

  printf("HAL_ADC_Start_DMA ---- ");
  Set_LED(USER_LED_GREEN,HIGH);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,4);
  printf("Success!\n");
  HAL_Delay(1000);
  Set_LED(USER_LED_GREEN,LOW);

  for(uint8_t i = 0;i < 4;i++){
    int continue_num = 0;
    printf("Check_ADC_Val_%d ---- ",i + 1);
    Set_LED(USER_LED_YELLOW,HIGH);
    while(!(adc_val_ch2[i] > 0)){
      continue_num ++;
      if(continue_num > 50){
        continue_num = 0;

        Set_LED(USER_LED_RED,HIGH);
        HAL_Delay(100);
        printf("FAIL!\n");
        printf("-- Restart_HAL_initialization\n");

        printf("-- HAL_ADC_Restart ---- ");
        HAL_Delay(100);
        HAL_ADC_Start(&hadc2);
        printf("Success!\n");

        printf("-- HAL_ADC_Restart_DMA ---- ");
        HAL_Delay(100);
        HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,4);
        printf("Success!\n");

        printf("Recheck_ADC_Val_%d ---- ",i + 1);
        HAL_Delay(100);
        Set_LED(USER_LED_RED,LOW);
      }
    }
    printf("Success!\n");
    HAL_Delay(100);
    Set_LED(USER_LED_YELLOW,LOW);
  }

  printf("*** Initalization Acomplished ***\n\n");
}


