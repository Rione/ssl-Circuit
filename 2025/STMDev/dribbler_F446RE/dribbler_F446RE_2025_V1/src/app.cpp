#include "app.hpp"
#include "adc.h"
#include "math.h"

uint16_t adc_val_ch1[4];
uint16_t sw_val = 0;

uint32_t IPf10ms_count = 1;
uint32_t IPf1ms_count = 1;

long int current_sum = 0;
uint8_t get_ball = 0;

CANBus can = CANBus(&hcan1, 0);
CANBus::CANData canRecvData;

Basic_IO_Control_Extension_Sensor Set_Sensor;
Basic_IO_Control_Motor Main_motor;
Basic_IO_Control_LED Set_LED;

void Setup(void){
  can.init();

  Set_LED.ALL_Control_EX_CAN(HIGH);
  HAL_Delay(500);
  Set_LED.ALL_Control_EX_CAN(LOW);
  HAL_Delay(500);

  Set_Administrator_Privilege();
  Set_ADC();
  // Set_DRV();
  // Set_Main_Motor();

  HAL_Delay(500);
  Set_LED.ALL_Control_EX_CAN(LOW);

  Set_Sensor.Ball_Sensor_Activate();
  Set_Sensor.ENC_Activate();
}

void MainLoop(){
  Set_LED.RED(HIGH);
}

void Set_Administrator_Privilege(){
  Set_LED.CAN_LED(HIGH);
  printf("*** Start Administrator Privilege Check ***\n");

  HAL_Delay(500);
  printf("Administrator Privilege ---- ");
  if(Administrator_Privilege == true){
    printf("Confirm!\n");
    printf("Switch to Administrator Mode\n");
    printf("Confirm Enforcement Processing\n");
  } else {
    printf("Unconfirm!\n");
    printf("Switch to Normal Mode\n");
  }

  printf("*** Administrator Privilege Check Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
}

void Set_ADC(){
  //ADC initialization
  Set_LED.CAN_LED(HIGH);

  printf("*** Start ADC Initalization ***\n");

  printf("HAL_ADC_Start ---- ");
  Set_LED.BLUE(HIGH);
  HAL_ADC_Start(&hadc1);
  printf("Success!\n");
  HAL_Delay(500);
  Set_LED.BLUE(LOW);

  printf("HAL_ADC_Start_DMA ---- ");
  Set_LED.GREEN(HIGH);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,4);
  printf("Success!\n");
  HAL_Delay(500);
  Set_LED.GREEN(LOW);

  for(uint8_t i = 0;i < 4;i++){
    int continue_num = 0;
    printf("Check_ADC_Val_%d ---- ",i + 1);
    Set_LED.YELLOW(HIGH);
    for(int j = 0;j < ADC_CONTINUE_NUM;j++){
      while(!(adc_val_ch1[i] > 0)){
        continue_num ++;
        if(continue_num > 50){
          continue_num = 0;
  
          Set_LED.RED(HIGH);
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
          Set_LED.RED(LOW);
        }
      }
    }
    printf("Success!\n");
    HAL_Delay(200);
    Set_LED.YELLOW(LOW);
  }

  printf("ADC Is Operating Normally\n");
  printf("*** ADC Initalization Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  Set_LED.YELLOW(HIGH);
  HAL_Delay(500);
}

void Set_ADC_Restart(){
  //ADC initialization
  Set_LED.LED_Flash_Activate = true;
  Set_LED.LED_Flash_RED_100ms = START;

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
  Set_LED.LED_Flash_RED_100ms = STOP;
  Set_LED.LED_Flash_Activate = false;
  Set_LED.RED(HIGH);
  HAL_Delay(500);
}

void Set_DRV(){
  //DRV initialization
  Set_LED.CAN_LED(HIGH);

  printf("*** Start Main Power Supply Check ***\n");
  Set_LED.BLUE(HIGH);

  printf("Main Power Supply ---- ");
  Main_motor.ENABLE();
  Main_motor.Forward(5);
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
    Set_LED.RED(HIGH);
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
          Set_ADC_Restart();
          printf("-- Recheck ADC Setup Acomplished\n");
          count = 1;
          ADC_Restart++;
        }
        count++;
      }
    }
    Set_LED.RED(LOW);
  }

  HAL_Delay(500);
  Main_motor.Brake();
  Main_motor.FET_DISABLE();
  Main_motor.DISABLE();

  printf("*** Main Power Supply Check Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  HAL_Delay(500);
}

void Set_Main_Motor(){
  //Main Motor initialization
  Set_LED.CAN_LED(HIGH);

  bool error_status = false; 

  printf("*** Start Main Motor Current Check ***\n");
  Set_LED.GREEN(HIGH);

  printf("Main Motor Forward Current ---- ");
  Main_motor.ENABLE();
  Main_motor.Forward(50);
  HAL_Delay(1000);
  int Forward_Current = 0;
  for(int i = 0;i < 50;i++){
    Forward_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(1);
  }
  Forward_Current /= 50;
  Main_motor.Brake();
  Main_motor.FET_DISABLE();
  Main_motor.DISABLE();
  if(Forward_Current > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    printf("Confirm!\n");
    printf("                           ---- Val = %d\n",Forward_Current);
  } else {
    error_status = true;
    Set_LED.RED(HIGH);
    printf("Unconfirm!\n");
    printf("                           ---- Val = %d\n",Forward_Current);
    printf("-- [CAUSION] : Main Motor Forward Current Is Not Normal\n");
    printf("-- [ADVICE] : Check Cancel Causion\n");
    while(sw_val != 0){
      HAL_Delay(50);
    }
    printf("-- Confirm Cancel Causion\n");
    Set_LED.RED(LOW);
  }

  Main_motor.ENABLE();
  Main_motor.Forward(30);
  HAL_Delay(10);
  Main_motor.Brake();
  Main_motor.FET_DISABLE();
  Main_motor.DISABLE();
  HAL_Delay(1000);

  printf("Main Motor Reverse Current ---- ");
  Main_motor.ENABLE();
  Main_motor.Reverse(50);
  HAL_Delay(1000);
  int Reverse_Current = 0;
  for(int i = 0;i < 50;i++){
    Reverse_Current += adc_val_ch1[MOTOR_CURRENT];
    HAL_Delay(1);
  }
  Reverse_Current /= 50;
  Main_motor.Brake();
  Main_motor.FET_DISABLE();
  Main_motor.DISABLE();
  if(Reverse_Current > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    printf("Confirm!\n");
    printf("                           ---- Val = %d\n",Reverse_Current);
  } else {
    error_status = true;
    Set_LED.RED(HIGH);
    printf("Unconfirm!\n");
    printf("                           ---- Val = %d\n",Reverse_Current);
    printf("-- [CAUSION] : Main Motor Reverse Current Is Not Normal\n");
    printf("-- [ADVICE] : Check Cancel Causion\n");
    while(sw_val != 0){
      HAL_Delay(50);
    }
    printf("-- Confirm Cancel Causion\n");
    Set_LED.RED(LOW);
  }

  printf("Check Current Value Differ ---- ");
  if(error_status == false){
    if(abs(Reverse_Current - Forward_Current) < 10){
      printf("Normal!\n");
      printf("Main Motor Current Is Operating Normally\n");
    } else {
      Set_LED.RED(HIGH);
      printf("Not Normal!\n");
      printf("-- CAUSION : Current Value Differ Is Too Big\n");
      printf("-- Check Cancel Causion\n");
      while(sw_val != 0){
        HAL_Delay(50);
      }
      printf("-- Confirm Cancel Causion\n");
      Set_LED.RED(LOW);
    }
  } else {
    printf("Skip!\n");
  }

  printf("*** Main Motor Current Check Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  HAL_Delay(500);
}

void Interrupt_Processing_f10ms(){
  //frq = 10ms
  //Control User Switch 
  sw_val = HAL_GPIO_ReadPin(USER_SW_GPIO_Port,USER_SW_Pin);

  //frq = 100ms
  if(IPf10ms_count % 10 == 0){
    IPf10ms_LED_Flash_Control();
  }

  IPf10ms_count++;
  if(IPf10ms_count == 100) IPf10ms_count = 1;
}

void Interrupt_Processing_f1ms(){
  //frq = 1ms
  current_sum += adc_val_ch1[MOTOR_CURRENT];

  //frq = 200ms
  if(IPf1ms_count % 200 == 0){
    HAL_CAN_Data_Output_ID0x1d2_466();
  }

  IPf1ms_count++;
  if(IPf1ms_count == 1000) IPf1ms_count = 1;
}

void IPf10ms_LED_Flash_Control(){
  //frq = 100ms
  //Control LED Flash
  if(Set_LED.LED_Flash_Activate == true){
    if(Set_LED.LED_Flash_RED_100ms != HOLD){
      if(Set_LED.LED_Flash_RED_100ms == START){
        HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
      } else if(Set_LED.LED_Flash_RED_100ms == STOP){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
        Set_LED.LED_Flash_RED_100ms = HOLD;
      }
    }
    if(Set_LED.LED_Flash_YELLOW_100ms != HOLD){
      if(Set_LED.LED_Flash_YELLOW_100ms == START){
        HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);
      } else if(Set_LED.LED_Flash_YELLOW_100ms == STOP){
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
        Set_LED.LED_Flash_YELLOW_100ms = HOLD;
      }
    }
    if(Set_LED.LED_Flash_BLUE_100ms != HOLD){
      if(Set_LED.LED_Flash_BLUE_100ms == START){
        HAL_GPIO_TogglePin(USER_LED3_GPIO_Port, USER_LED3_Pin);
      } else if(Set_LED.LED_Flash_BLUE_100ms == STOP){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
        Set_LED.LED_Flash_BLUE_100ms = HOLD;
      }
    }
    if(Set_LED.LED_Flash_GREEN_100ms != HOLD){
      if(Set_LED.LED_Flash_GREEN_100ms == START){
        HAL_GPIO_TogglePin(USER_LED4_GPIO_Port, USER_LED4_Pin);
      } else if(Set_LED.LED_Flash_GREEN_100ms == STOP){
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
        Set_LED.LED_Flash_GREEN_100ms = HOLD;
      }
    }
    if(Set_LED.LED_Flash_CAN_100ms != HOLD){
      if(Set_LED.LED_Flash_CAN_100ms == START){
        HAL_GPIO_TogglePin(CAN_LED_GPIO_Port, CAN_LED_Pin);
      } else if(Set_LED.LED_Flash_CAN_100ms == STOP){
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
        Set_LED.LED_Flash_CAN_100ms = HOLD;
      }
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (can.getHcan() == hcan){
    Set_LED.CAN_LED(HIGH);
    can.recv(canRecvData);
    switch (canRecvData.stdId){
    case 0x1d1: //465
      HAL_CAN_Data_Input_ID0x1d1_465();
      break;
    default:
      break;
    }
    Set_LED.CAN_LED(LOW);
  }
}

void HAL_CAN_Data_Output_ID0x1d2_466(){
  uint16_t current_val = current_sum / 200;
  uint8_t photo = 0;
  uint8_t current = 0;

  if (current_val > 150)
    current = 1;
  if (adc_val_ch1[BALL_SENSOR_VAL] < 100)
    photo = 1;

  if (current == 1 && get_ball == 0){
    Set_LED.GREEN(HIGH);
    get_ball = 1;
  }
  else if (photo == 0 && get_ball == 1){
    Set_LED.GREEN(LOW);
    get_ball = 0;
  }

  Set_LED.CAN_LED(HIGH);

  CANBus::CANData data = {
    .stdId = 0x1d2,
    .data = {
      get_ball,
      current,
      (uint8_t)(current_sum & 0xFF),
      (uint8_t)((current_sum >> 8) & 0xFF),
      photo,
      (uint8_t)(adc_val_ch1[BALL_SENSOR_VAL] & 0xFF),
      (uint8_t)((adc_val_ch1[BALL_SENSOR_VAL] >> 8) & 0xFF),
    },
  };
  can.send(data);

  Set_LED.CAN_LED(LOW);

  current_sum = 0;
}

void HAL_CAN_Data_Input_ID0x1d1_465(){
  Main_motor.ENABLE();
  Main_motor.Forward(canRecvData.data[0]);
  Set_LED.BLUE(HIGH);
}





