#include "app.hpp"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

uint16_t adc_val_ch1[5] = {0}; // Array to hold ADC values

void Setup(void){

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(500);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_val_ch1, 5);
  //HAL_ADC_Start(&hadc1);
  
  HAL_GPIO_WritePin(SVC_EN_GPIO_Port, SVC_EN_Pin, GPIO_PIN_SET); // Enable SVC
  
  


  HAL_GPIO_WritePin(MVC_DIR_GPIO_Port, MVC_DIR_Pin, GPIO_PIN_RESET); 

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 750);
  
  //HAL_GPIO_WritePin(MVC_EN1_GPIO_Port, MVC_EN1_Pin, GPIO_PIN_SET); // Enable MVC1
  //HAL_GPIO_WritePin(MVC_EN2_GPIO_Port, MVC_EN2_Pin, GPIO_PIN_SET); // Enable MVC2

}

void MainLoop(){
  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700); 




  printf("%u,%u,%u,%u,%u\n", adc_val_ch1[0], adc_val_ch1[1], adc_val_ch1[2], adc_val_ch1[3], adc_val_ch1[4]);
  HAL_Delay(100);
}

