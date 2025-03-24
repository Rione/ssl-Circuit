//AD_Setup_Control

#include "AD_Setup_Control.hpp"

Basic_IO_Control_Extension_Sensor ADSC_Sensor;
Basic_IO_Control_Motor ADSC_Motor;
Basic_IO_Control_LED ADSC_LED;

void AD_Setup_Control::Administrator_Privilege(){
  // printf("*** Start Administrator Privilege Check ***\n");

  // HAL_Delay(500);
  // printf("Administrator Privilege ---- ");
  // if(Fnc_Administrator_Privilege == true){
  //     printf("Confirm!\n");
  //     printf("Switch to Administrator Mode\n");
  //     printf("Confirm Enforcement Processing\n");
  // } else {
  //     printf("Unconfirm!\n");
  //     printf("Switch to Normal Mode\n");
  // }

  // printf("*** Administrator Privilege Check Acomplished ***\n\n");
}

void AD_Setup_Control::ADC_Check(){
  //ADC initialization
  int hal_restart_tim = -1;
  bool hal_restart = false;

  printf("*** Start ADC Initalization ***\n");

  do{
    hal_restart = false;
    hal_restart_tim ++;

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

    for(int i = 0;i < 4;i++){
      int continue_num = 0;
      printf("Check_ADC_Val_%d ---- ",i + 1);
      ADSC_LED.YELLOW(HIGH);
      while(!(adc_val_ch1[i] > 0)){
        HAL_Delay(10);
        continue_num ++;
        if(continue_num > 50){
          printf("FAIL!\n");
          if(hal_restart_tim > 6){
            HAL_NVIC_SystemReset();
          } else {
            printf("-- Restart_HAL_initialization\n");
            ADSC_LED.RED(HIGH);
            hal_restart = true; 
          }
        }
      }

      if(hal_restart == false){
        printf("Success!\n");
        ADSC_LED.YELLOW(LOW);
        ADSC_LED.RED(LOW);
      }

      HAL_Delay(200);
    }
  }while(hal_restart != false);

  printf("ADC Is Operating Normally\n");
  printf("*** ADC Initalization Acomplished ***\n\n");

  ADSC_LED.YELLOW(HIGH);
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
    current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(10);
  } 
  printf("%d\n",current / 50);
  if((current / 50) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    current = 0;
    printf("Confirm!\n");
    printf("Main Power Supply Is Operating Normally\n");
  } else {
    // printf("Unconfirm!\n");
    // printf("-- [CAUSION] : Main Power Supply Is Too LOW\n");
    // printf("-- [ADVICE] : Cancel Causion or Establish Power Supply\n");
    // ADSC_LED.RED(HIGH);
    // int count = 1;
    // int ADC_Restart = 0;
    // for(;;){
    //   current = 0;
    //   for(int i = 0;i < 10;i++){
    //     HAL_Delay(5);
    //     current += adc_val_ch1[MOTOR_CURRENT];
    //   } 
    //   if((current / 10) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    //     printf("Recheck Main Power Supply ---- ");
    //     printf("Confirm!\n");
    //     printf("Main Power Supply Is Operating Normally\n");
    //     break;
    //   }
    //   if(sw_val == 0){
    //     printf("Recheck Main Power Supply ---- ");
    //     printf("Unconfirm!\n");
    //     printf("-- Confirm Cancel Causion\n");
    //     break;
    //   }
    //   if(Recheck_ADC_Setup == true){
    //     if(count > 100){
    //       if(ADC_Restart == 0){
    //         printf("Recheck Main Power Supply ---- ");
    //         printf("Unconfirm!\n");
    //       }
    //       printf("-- Recheck ADC Setup\n");
    //       //Set_ADC_Restart();
    //       printf("-- Recheck ADC Setup Acomplished\n");
    //       count = 1;
    //       ADC_Restart++;
    //     }
    //     count++;
    //   }
    // }
    // ADSC_LED.RED(LOW);
    HAL_NVIC_SystemReset();
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
  ADSC_Motor.Forward(80);
  HAL_Delay(1000);
  long int Forward_Current = 0;
  for(int i = 0;i < 100;i++){
    Forward_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(5);
  }
  Forward_Current /= 100;
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

  ADSC_Motor.Brake();
  ADSC_Motor.FET_DISABLE();
  ADSC_Motor.DISABLE();
  HAL_Delay(1000);

  printf("Main Motor Reverse Current ---- ");
  ADSC_Motor.ENABLE();
  ADSC_Motor.Reverse(80);
  HAL_Delay(1000);
  long int Reverse_Current = 0;
  for(int i = 0;i < 100;i++){
    Reverse_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(5);
  }
  Reverse_Current /= 100;
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
    if(abs(Reverse_Current - Forward_Current) < 20){
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



