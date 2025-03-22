//AD_Setup_Control

#include "AD_Setup_Control.hpp"

Basic_IO_Control_Extension_Sensor ADSC_Sensor;
Basic_IO_Control_Motor ADSC_Motor;
Basic_IO_Control_LED ADSC_LED;

void AD_Setup_Control::Administrator_Privilege(){
  ADSC_LED.CAN_LED(HIGH);
  printf("*** Start Administrator Privilege Check ***\n");

  HAL_Delay(500);
  printf("Administrator Privilege ---- ");
  if(Fnc_Administrator_Privilege == true){
      printf("Confirm!\n");
      printf("Switch to Administrator Mode\n");
      printf("Confirm Enforcement Processing\n");
  } else {
      printf("Unconfirm!\n");
      printf("Switch to Normal Mode\n");
  }

  printf("*** Administrator Privilege Check Acomplished ***\n\n");
  ADSC_LED.CAN_LED(LOW);
}

void AD_Setup_Control::ADC_Check(){
  //ADC initialization
  ADSC_LED.CAN_LED(HIGH);

  printf("*** Start ADC Initalization ***\n");

  printf("HAL_ADC_Start ---- ");
  ADSC_LED.BLUE(HIGH);
  HAL_ADC_Start(&hadc1);
  printf("Success!\n");
  HAL_Delay(500);
  ADSC_LED.BLUE(LOW);

  printf("HAL_ADC_Start_DMA ---- ");
  ADSC_LED.GREEN(HIGH);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
  printf("Success!\n");
  HAL_Delay(500);
  ADSC_LED.GREEN(LOW);

  for(uint8_t i = 0;i < 4;i++){
    int continue_num = 0;
    printf("Check_ADC_Val_%d ---- ",i + 1);
    ADSC_LED.YELLOW(HIGH);
    for(int j = 0;j < ADC_CONTINUE_NUM;j++){
      while(!(adc_val_ch1[i] > 0)){
        continue_num ++;
        if(continue_num > 50){
          continue_num = 0;
  
          ADSC_LED.RED(HIGH);
          HAL_Delay(100);
          printf("FAIL!\n");
          printf("-- Restart_HAL_initialization\n");
  
          printf("-- HAL_ADC_Restart ---- ");
          HAL_Delay(100);
          HAL_ADC_Start(&hadc1);
          printf("Success!\n");
  
          printf("-- HAL_ADC_Restart_DMA ---- ");
          HAL_Delay(100);
          HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
          printf("Success!\n");
  
          printf("Recheck_ADC_Val_%d ---- ",i + 1);
          HAL_Delay(100);
          ADSC_LED.RED(LOW);
        }
      }
    }
    printf("Success!\n");
    HAL_Delay(200);
    ADSC_LED.YELLOW(LOW);
  }

  printf("ADC Is Operating Normally\n");
  printf("*** ADC Initalization Acomplished ***\n\n");
  ADSC_LED.CAN_LED(LOW);
  ADSC_LED.YELLOW(HIGH);
  HAL_Delay(500);
}


