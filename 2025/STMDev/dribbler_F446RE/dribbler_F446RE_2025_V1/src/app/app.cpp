#include "app.hpp"
// #include "AnalogIn.hpp"

// AnalogIn amp_sense(&hadc,ADC_CHANNEL_3);

#define RED 0
#define BLUE 1

#define FORWARD 0
#define REVERSE 1
#define BRAKE 2
#define FET_DISABLE 3;

#define DRV_ENABLE HAL_GPIO_WritePin(MD_nSEEP_GPIO_Port, MD_nSEEP_Pin, GPIO_PIN_SET)
#define DRV_DISABLE HAL_GPIO_WritePin(MD_nSEEP_GPIO_Port, MD_nSEEP_Pin, GPIO_PIN_RESET)

#define HIGH 0
#define LOW 1 

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


    // printf("%d\n",amp_sense.read())
    // HAL_Delay(3);
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
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, map());
  }
}