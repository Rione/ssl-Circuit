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

class LED_Control {
  public:
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
};

Motor_Control Main_motor;
LED_Control Set_LED;

void Setup(void){
  Set_LED.ALL_Control(HIGH);
  HAL_Delay(500);
  Set_LED.ALL_Control(LOW);

  ADC_Setup();
  DRV_Setup();

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

    // printf("in\n");

    

    Main_motor.Forward(50);
    // HAL_Delay(50);
    // Main_motor.Brake();
    // HAL_Delay(200);
    // Main_motor.Reverse(100);
    // HAL_Delay(200);
    // Main_motor.Brake();
    // HAL_Delay(200);
    int val = 0;
    for(int i = 0;i < 100;i++){
      val += adc_val_ch2[0];
      HAL_Delay(1);
    }
    printf("%d\n",val / 100);
  }
}

void ADC_Setup(){
  //ADC initialization
  Set_LED.CAN_LED(HIGH);

  printf("\n");
  printf("*** Start ADC Initalization ***\n");

  printf("HAL_ADC_Start ---- ");
  Set_LED.BLUE(HIGH);
  HAL_ADC_Start(&hadc2);
  printf("Success!\n");
  HAL_Delay(500);
  Set_LED.BLUE(LOW);

  printf("HAL_ADC_Start_DMA ---- ");
  Set_LED.GREEN(HIGH);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adc_val_ch2,4);
  printf("Success!\n");
  HAL_Delay(500);
  Set_LED.GREEN(LOW);

  for(uint8_t i = 0;i < 4;i++){
    int continue_num = 0;
    printf("Check_ADC_Val_%d ---- ",i + 1);
    Set_LED.YELLOW(HIGH);
    while(!(adc_val_ch2[i] > 0)){
      continue_num ++;
      if(continue_num > 50){
        continue_num = 0;

        Set_LED.RED(HIGH);
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
        Set_LED.RED(LOW);

        HAL_Delay(1000);
      }
    }
    printf("Success!\n");
    HAL_Delay(100);
    Set_LED.YELLOW(LOW);
  }

  printf("*** ADC Initalization Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  Set_LED.YELLOW(HIGH);
  HAL_Delay(500);
}

void DRV_Setup(){
  //ADC initialization
  Set_LED.CAN_LED(HIGH);

  printf("*** Start Main Power Supply Check ***\n");
  Set_LED.BLUE(HIGH);

  Main_motor.ENABLE();
  Main_motor.Forward(70);
  HAL_Delay(100);
  int current = 0;
  for(int i = 0;i < 50;i++){
    HAL_Delay(10);
    current += adc_val_ch2[MOTOR_CURRENT];
  } 
  if((current / 50) > DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE){
    current = 0;
    printf("Main Power Supply Is Operating Normally\n");
  } else {
    printf("CAUSION : Main Power Supply Is Too LOW\n");
    printf("-- Cancel Causion or Conect Power Supply\n");
    Set_LED.RED(HIGH);
    do{
      current = 0;
      for(int i = 0;i < 10;i++){
        HAL_Delay(5);
        current += adc_val_ch2[MOTOR_CURRENT];
      } 
    } while((current / 10) < DRV_MIN_CURRENT - DRV_MIN_CURRENT_MINUS_RANGE);
    Set_LED.RED(LOW);
    printf("-- Confirm Main Power Supply\n");
    printf("Main Power Supply Is Operating Normally\n");
    //add extra code
  }
  HAL_Delay(500);
  Main_motor.Brake();
  Main_motor.DISABLE();

  printf("*** Main Power Supply Check Acomplished ***\n\n");
  Set_LED.CAN_LED(LOW);
  HAL_Delay(500);
}
