//AD_Setup_Control

#include "IO_Setup_Control.hpp"

Basic_IO_Control_Extension_Sensor ADSC_Sensor;
Basic_IO_Control_Motor ADSC_Motor;
Basic_IO_Control_LED ADSC_LED;
extern Basic_IO_Control_LED_Flash IPf100ms_Flash;

bool Enforcement_Processing;

void AD_Setup_Control::ADC_Check(){
  //ADC initialization
  int hal_restart_tim = -1;
  bool hal_restart = false;

  printf("\n*** Start ADC Initalization ***\n");

  do{
    hal_restart = false;
    hal_restart_tim ++;

    printf("HAL ADC Start ---- ");
    ADSC_LED.BLUE(HIGH);
    HAL_ADC_Start(&hadc1);
    printf("Success!\n");
    HAL_Delay(500);
    ADSC_LED.BLUE(LOW);

    printf("HAL ADC Start DMA ---- ");
    ADSC_LED.GREEN(HIGH);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
    printf("Success!\n");
    HAL_Delay(500);
    ADSC_LED.GREEN(LOW);

    for(int i = 0;i < 4;i++){
      int continue_num = 0;
      printf("Check ADC_Val_%d ---- ",i + 1);
      ADSC_LED.YELLOW(HIGH);
      while(!(adc_val_ch1[i] > 0)){
        HAL_Delay(10);
        continue_num ++;
        if(continue_num > 50){
          printf("FAIL!\n");
          ADSC_LED.RED(HIGH);
          if(hal_restart_tim >= 3){
            printf("\n[][][] Inisiate System Rest [][][]\n");
            HAL_NVIC_SystemReset();
          } else {
            printf("-- Restart HAL initialization\n");
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
  } while(hal_restart != false);

  printf("ADC Is Operating Normally\n");
  printf("*** ADC Initalization Acomplished ***\n");

  ADSC_LED.YELLOW(HIGH);
  HAL_Delay(500);
}

void AD_Setup_Control::Administrator_Privilege(){
  printf("\n*** Start Administrator Privilege Check ***\n");
  printf("Administrator Privilege ---- ");
  if(Fnc_Administrator_Privilege == true){
    printf("Confirm!\n");
    printf("Switch to Administrator Mode\n");
    printf("Confirm Enforcement Processing\n");
    Enforcement_Processing = true;
  } else {
    printf("Unconfirm!\n");
    printf("Switch to Normal Mode\n");
    Enforcement_Processing = false;
  }

  printf("*** Administrator Privilege Check Acomplished ***\n");

  HAL_Delay(500);
}

void AD_Setup_Control::DRV_Check(){
  //DRV initialization
  int DRV_restart_tim = -1;
  bool DRV_restart = false;

  const int Main_Power_max = Main_Power_Constant + Main_Power_Constant_Range;
  const int Main_Power_min = Main_Power_Constant - Main_Power_Constant_Range;
  
  printf("\n*** Start Main Power Supply Check ***\n");
  IPf100ms_Flash.LED_Flash_BLUE = START;

  do {
    int current = 0;
    DRV_restart = false;
    DRV_restart_tim ++;

    printf("Main Power Supply ---- ");
    ADSC_Motor.ENABLE();
    ADSC_Motor.Forward(5);
    HAL_Delay(1000);

    for(int i = 0;i < 50;i++){
      current += adc_val_ch1[MOTOR_CURRENT];
      HAL_Delay(10);
    } 

    if((current / 50) > Main_Power_min && (current / 50) < Main_Power_max){
      printf("Confirm! (Val = %d)\n",current / 50);
      printf("Main Power Supply Is Operating Normally\n");
    } else {
      ADSC_LED.RED(HIGH);
      printf("Unconfirm! (Val = %d)\n",current / 50);
      printf("-- Main Power Supply Is Too LOW\n");
      DRV_restart = true;

      if(Enforcement_Processing == true){
        if(DRV_restart_tim >= 3){
          printf("\n[][][] Inisiate System Rest [][][]\n");
          HAL_NVIC_SystemReset();
        } else {
          printf("-- Restart Main Power Supply Check\n");
        }
      } else {
        if(sw_val == 0){
          DRV_restart = false;
          printf("Confirm Cancel Causion\n");
        } else printf("-- Restart Main Power Supply Check\n");
      }
    }
  } while(DRV_restart != false);

  ADSC_LED.RED(LOW);

  HAL_Delay(500);
  ADSC_Motor.Brake();
  ADSC_Motor.DISABLE();

  IPf100ms_Flash.LED_Flash_BLUE = STOP;
  HAL_Delay(100);
  ADSC_LED.BLUE(HIGH);

  printf("*** Main Power Supply Check Acomplished ***\n");
  HAL_Delay(500);
}

void AD_Setup_Control::Motor_Check(){
  //Main Motor initialization
  int motor_restart_tim = -1;
  bool motor_restart = false;

  Motor_Ajust_Value = 0;

  printf("\n*** Start Main Motor Check ***\n");
  IPf100ms_Flash.LED_Flash_GREEN = START;
  ADSC_Motor.ENABLE();

  do{
    const int motor_current_max = Motor_Base_Current + Motor_Ajust_Value + Motor_Base_Current_RANGE;
    const int motor_current_min = Motor_Base_Current + Motor_Ajust_Value - Motor_Base_Current_RANGE;

    int Forward_Current = 0;
    int Reverse_Current = 0;

    motor_restart = false;
    motor_restart_tim ++;

    ADSC_Motor.Forward(100);
    HAL_Delay(500);
    for(int i = 0;i < 100;i++){
      Forward_Current += adc_val_ch1[MOTOR_CURRENT];
      HAL_Delay(2);
    }
    Forward_Current /= 100;
    ADSC_Motor.Brake();
    HAL_Delay(500);
    ADSC_Motor.Reverse(100);
    HAL_Delay(500);
    for(int i = 0;i < 100;i++){
      Reverse_Current += adc_val_ch1[MOTOR_CURRENT];
      HAL_Delay(2);
    }
    Reverse_Current /= 100;
    ADSC_Motor.Brake();

    printf("Main Motor Forward Current ---- ");
    if(Forward_Current > motor_current_min && Forward_Current < motor_current_max){
      printf("Confirm! (Val = %d)\n",Forward_Current);
    } else {
      printf("Unconfirm! (Val = %d)\n",Forward_Current);
      printf("-- Main Motor Forward Current Is Not Normal\n");
      motor_restart = true;
    }
  
    printf("Main Motor Reverse Current ---- ");
    if(Reverse_Current > motor_current_min && Reverse_Current < motor_current_max){
      printf("Confirm! (Val = %d)\n",Reverse_Current);
    } else {
      printf("Unconfirm! (Val = %d)\n",Reverse_Current);
      printf("-- Main Motor Reverse Current Is Not Normal\n");
      motor_restart = true;
    }
  
    printf("Check Current Value Differ ---- ");
    if(abs(Forward_Current - Reverse_Current) < Motor_Current_Differ_Tolerance){
      printf("Confirm! (Val = %d)\n",abs(Forward_Current - Reverse_Current));
    } else {
      printf("Unconfirm! (Val = %d)\n",abs(Forward_Current - Reverse_Current));
      printf("-- Current Value Differ Is Not Normal\n");
      motor_restart = true;
    }

    Motor_Ajust_Value = Forward_Current - Motor_Base_Current;
    printf("-- Motor Value Ajusted! (Val = %d)\n",Motor_Ajust_Value);
    
    if(motor_restart == true){
      ADSC_LED.RED(HIGH);
      if(Enforcement_Processing == true){
        if(motor_restart_tim >= 3){
          printf("\n[][][] Inisiate System Rest [][][]\n");
          HAL_NVIC_SystemReset();
        } else {
          printf("-- Restart Main Motor Check\n");
          HAL_Delay(1000);
        }
      } else {
        if(sw_val == 0){
          motor_restart = false;
          printf("Confirm Cancel Causion\n");
        } else {
          printf("-- Restart Main Motor Check\n");
          HAL_Delay(1000);
        }
      }
    }
  }while(motor_restart != false);

  ADSC_LED.RED(LOW);
  ADSC_Motor.Brake();
  ADSC_Motor.DISABLE();

  IPf100ms_Flash.LED_Flash_GREEN = STOP;
  HAL_Delay(100);
  ADSC_LED.GREEN(HIGH);

  printf("Main Motor Is Operating Normally\n");
  printf("*** Main Motor Check Acomplished ***\n\n");
  HAL_Delay(500);
}



