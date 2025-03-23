//AD_Setup_Control

#include "AD_Setup_Control.hpp"

Basic_IO_Control_Extension_Sensor ADSC_Sensor;
Basic_IO_Control_Motor ADSC_Motor;
Basic_IO_Control_LED ADSC_LED;

extern uint16_t sw_val;
extern uint16_t adc_val_ch1[4];

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

void AD_Setup_Control::ADC_Recheck(){
  //ADC initialization
  ADSC_LED.LED_Flash_Activate = true;
  ADSC_LED.LED_Flash_RED_100ms = START;

  printf("[ADC] HAL_ADC_Start ---- ");
  HAL_ADC_Start(&hadc1);
  printf("Success!\n");
  HAL_Delay(500);

  printf("[ADC] HAL_ADC_Start_DMA ---- ");
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
  printf("Success!\n");
  HAL_Delay(500);

  for(uint8_t i = 0;i < 4;i++){
    int continue_num = 0;
    printf("[ADC] Check_ADC_Val_%d ---- ",i + 1);
    for(int j = 0;j < ADC_CONTINUE_NUM;j++){
      while(!(adc_val_ch1[i] > 0)){
        continue_num ++;
        if(continue_num > 50){
          continue_num = 0;
  
          HAL_Delay(100);
          printf("FAIL!\n");
          printf("[ADC] -- Restart_HAL_initialization\n");
  
          printf("[ADC] -- HAL_ADC_Restart ---- ");
          HAL_Delay(100);
          HAL_ADC_Start(&hadc1);
          printf("Success!\n");
  
          printf("[ADC] -- HAL_ADC_Restart_DMA ---- ");
          HAL_Delay(100);
          HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
          printf("Success!\n");
  
          printf("[ADC] Recheck_ADC_Val_%d ---- ",i + 1);
          HAL_Delay(100);
        }
      }
    }
    printf("Success!\n");
    HAL_Delay(200);
  }
  ADSC_LED.LED_Flash_RED_100ms = STOP;
  ADSC_LED.LED_Flash_Activate = false;
  ADSC_LED.RED(HIGH);
  HAL_Delay(500);
}

void AD_Setup_Control::DRV_Check(){
  //DRV initialization
  ADSC_LED.CAN_LED(HIGH);

  printf("*** Start Main Power Supply Check ***\n");
  ADSC_LED.BLUE(HIGH);

  printf("Main Power Supply ---- ");
  ADSC_Motor.ENABLE();
  ADSC_Motor.Forward(5);
  HAL_Delay(100);
  int current = 0;
  for(int i = 0;i < 50;i++){
    HAL_Delay(10);
    current += adc_val_ch1[MOTOR_CURRENT];
  } 
  if((current / 50) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    current = 0;
    printf("Confirm!\n");
    printf("Main Power Supply Is Operating Normally\n");
  } else {
    printf("Unconfirm!\n");
    printf("-- [CAUSION] : Main Power Supply Is Too LOW\n");
    printf("-- [ADVICE] : Cancel Causion or Establish Power Supply\n");
    ADSC_LED.RED(HIGH);
    int count = 1;
    int ADC_Restart = 0;
    for(;;){
      current = 0;
      for(int i = 0;i < 10;i++){
        HAL_Delay(5);
        current += adc_val_ch1[MOTOR_CURRENT];
      } 
      if((current / 10) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
        printf("Recheck Main Power Supply ---- ");
        printf("Confirm!\n");
        printf("Main Power Supply Is Operating Normally\n");
        break;
      }
      if(sw_val == 0){
        printf("Recheck Main Power Supply ---- ");
        printf("Unconfirm!\n");
        printf("-- Confirm Cancel Causion\n");
        break;
      }
      if(Recheck_ADC_Setup == true){
        if(count > 100){
          if(ADC_Restart == 0){
            printf("Recheck Main Power Supply ---- ");
            printf("Unconfirm!\n");
          }
          printf("-- Recheck ADC Setup\n");
          //Set_ADC_Restart();
          printf("-- Recheck ADC Setup Acomplished\n");
          count = 1;
          ADC_Restart++;
        }
        count++;
      }
    }
    ADSC_LED.RED(LOW);
  }

  HAL_Delay(500);
  ADSC_Motor.Brake();
  ADSC_Motor.FET_DISABLE();
  ADSC_Motor.DISABLE();

  printf("*** Main Power Supply Check Acomplished ***\n\n");
  ADSC_LED.CAN_LED(LOW);
  HAL_Delay(500);
}

void AD_Setup_Control::Motor_Check(){
  //Main Motor initialization
  ADSC_LED.CAN_LED(HIGH);

  bool error_status = false; 

  printf("*** Start Main Motor Current Check ***\n");
  ADSC_LED.GREEN(HIGH);

  printf("Main Motor Forward Current ---- ");
  ADSC_Motor.ENABLE();
  ADSC_Motor.Forward(50);
  HAL_Delay(1000);
  int Forward_Current = 0;
  for(int i = 0;i < 50;i++){
    Forward_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(1);
  }
  Forward_Current /= 50;
  ADSC_Motor.Brake();
  ADSC_Motor.FET_DISABLE();
  ADSC_Motor.DISABLE();
  if(Forward_Current > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    printf("Confirm!\n");
    printf("                           ---- Val = %d\n",Forward_Current);
  } else {
    error_status = true;
    ADSC_LED.RED(HIGH);
    printf("Unconfirm!\n");
    printf("                           ---- Val = %d\n",Forward_Current);
    printf("-- [CAUSION] : Main Motor Forward Current Is Not Normal\n");
    printf("-- [ADVICE] : Check Cancel Causion\n");
    while(sw_val != 0){
      HAL_Delay(50);
    }
    printf("-- Confirm Cancel Causion\n");
    ADSC_LED.RED(LOW);
  }

  ADSC_Motor.ENABLE();
  ADSC_Motor.Forward(30);
  HAL_Delay(10);
  ADSC_Motor.Brake();
  ADSC_Motor.FET_DISABLE();
  ADSC_Motor.DISABLE();
  HAL_Delay(1000);

  printf("Main Motor Reverse Current ---- ");
  ADSC_Motor.ENABLE();
  ADSC_Motor.Reverse(50);
  HAL_Delay(1000);
  int Reverse_Current = 0;
  for(int i = 0;i < 50;i++){
    Reverse_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(1);
  }
  Reverse_Current /= 50;
  ADSC_Motor.Brake();
  ADSC_Motor.FET_DISABLE();
  ADSC_Motor.DISABLE();
  if(Reverse_Current > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    printf("Confirm!\n");
    printf("                           ---- Val = %d\n",Reverse_Current);
  } else {
    error_status = true;
    ADSC_LED.RED(HIGH);
    printf("Unconfirm!\n");
    printf("                           ---- Val = %d\n",Reverse_Current);
    printf("-- [CAUSION] : Main Motor Reverse Current Is Not Normal\n");
    printf("-- [ADVICE] : Check Cancel Causion\n");
    while(sw_val != 0){
      HAL_Delay(50);
    }
    printf("-- Confirm Cancel Causion\n");
    ADSC_LED.RED(LOW);
  }

  printf("Check Current Value Differ ---- ");
  if(error_status == false){
    if(abs(Reverse_Current - Forward_Current) < 10){
      printf("Normal!\n");
      printf("Main Motor Current Is Operating Normally\n");
    } else {
      ADSC_LED.RED(HIGH);
      printf("Not Normal!\n");
      printf("-- CAUSION : Current Value Differ Is Too Big\n");
      printf("-- Check Cancel Causion\n");
      while(sw_val != 0){
        HAL_Delay(50);
      }
      printf("-- Confirm Cancel Causion\n");
      ADSC_LED.RED(LOW);
    }
  } else {
    printf("Skip!\n");
  }

  printf("*** Main Motor Current Check Acomplished ***\n\n");
  ADSC_LED.CAN_LED(LOW);
  HAL_Delay(500);
}



