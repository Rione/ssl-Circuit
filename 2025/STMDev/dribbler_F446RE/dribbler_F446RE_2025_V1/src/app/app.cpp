#include "app.hpp"
#include "adc.h"
#include "config.h"
#include "math.h"

uint16_t adc_val_ch1[4];
uint16_t sw_val = 0;

uint16_t IPf10ms_count = 1;

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

class LED_Control {
  public:
    uint8_t LED_Flash_RED_100ms = hold;
    uint8_t LED_Flash_YELLOW_100ms = hold;
    uint8_t LED_Flash_BLUE_100ms = hold;
    uint8_t LED_Flash_GREEN_100ms = hold;
    uint8_t LED_Flash_CAN_100ms = hold;

    void RED(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
      }
    };

    void YELLOW(int status) {
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
      }
    };

    void BLUE(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
      }
    };

    void GREEN(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
      }
    };

    void CAN_LED(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
      }
    };

    void ALL_Control(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
      }
    };

    void ALL_Control_EX_CAN(int status){
      if(status == HIGH){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
      } else if(status == LOW){
        HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
      }
    }
};

Motor_Control Main_motor;
LED_Control Set_LED;

void Setup(void){
  Set_LED.ALL_Control(HIGH);
  HAL_Delay(500);
  Set_LED.ALL_Control(LOW);

  ADC_Setup();
  DRV_Setup();
  Main_Motor_Setup();

  HAL_Delay(500);
  Set_LED.ALL_Control(LOW);
  Set_LED.GREEN(HIGH);
}

void MainLoop(){
  while(1){
    // uint16_t thr = 750;
    // CANBus::CANData data = {
    //     .stdId = 0x09,
    //     .data = {
    //         (uint8_t)(thr & 0xFF),
    //         (uint8_t)((thr >> 8) & 0xFF),
    //     },
    // };
    // can.send(data);

    int t = 1,p = 0;
    Main_motor.Forward(30);
    for(int i = 0;i < 50;i++){
      p += adc_val_ch1[MOTOR_CURRENT];
      HAL_Delay(t);
    }
    //printf("%d\n",p / 50);
    // Main_motor.Brake();
    // HAL_Delay(t);
    // Main_motor.Reverse(80);
    // HAL_Delay(t);
    // Main_motor.Brake();
    // HAL_Delay(t);
  }
}

void ADC_Setup(){
  //ADC initialization
  Set_LED.CAN_LED(HIGH);

  printf("\n");
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

  printf("*** ADC Initalization Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  Set_LED.YELLOW(HIGH);
  HAL_Delay(500);
}

void DRV_Setup(){
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
    printf("-- CAUSION : Main Power Supply Is Too LOW\n");
    printf("-- Cancel Causion or Establish Power Supply\n");
    printf("Recheck Main Power Supply ---- ");
    Set_LED.RED(HIGH);
    for(;;){
      current = 0;
      for(int i = 0;i < 10;i++){
        HAL_Delay(5);
        current += adc_val_ch1[MOTOR_CURRENT];
      } 
      if((current / 10) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
        printf("Confirm!\n");
        printf("Main Power Supply Is Operating Normally\n");
        break;
      }
      if(sw_val == 0){
        printf("Unconfirm!\n");
        printf("-- Confirm Cancel Causion\n");
        break;
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

void Main_Motor_Setup(){
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
    printf("-- CAUSION : Main Motor Forward Current Is Not Normal\n");
    printf("-- Check Cancel Causion\n");
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
    printf("-- CAUSION : Main Motor Reverse Current Is Not Normal\n");
    printf("-- Check Cancel Causion\n");
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
    //Control LED Flash
    if(Set_LED.LED_Flash_RED_100ms == true)
  }

  IPf10ms_count++;
  if(IPf10ms_count > 100) IPf10ms_count = 1;
}

void Interrupt_Processing_f100ns(){
  //extra code
}




