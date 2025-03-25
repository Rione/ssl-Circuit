#include "app.hpp"
#include "adc.h"
#include "math.h"

uint16_t sw_val = 0;
uint16_t adc_val_ch1[4];
uint32_t IPf10ms_count = 1;
uint32_t IPf1ms_count = 1;
long int current_sum = 0;
uint8_t get_ball = 0;

CANBus can = CANBus(&hcan1, 0);
CANBus::CANData canRecvData;
bool Halt_CAN_Data_Send;

Basic_IO_Control_Extension_Sensor Set_Sensor;
Basic_IO_Control_Motor Main_motor;
Basic_IO_Control_LED Set_LED;

AD_Setup_Control AD_Setup;

void Setup(void){
  Halt_CAN_Data_Send = true;

  HAL_Delay(500);
  Set_LED.ALL_Control(HIGH);
  HAL_Delay(1000);
  Set_LED.ALL_Control(LOW);
  HAL_Delay(500);

  Set_LED.LED_Flash_Activate = true;
  Set_LED.LED_Flash_CAN_100ms = START;
  AD_Setup.ADC_Check();
  AD_Setup.Administrator_Privilege();
  AD_Setup.DRV_Check();
  AD_Setup.Motor_Check();
  Set_LED.LED_Flash_CAN_100ms = STOP;
  Set_LED.LED_Flash_Activate = false;

  HAL_Delay(500);
  Set_LED.ALL_Control_EX_CAN(LOW);

  can.init();
  Set_Sensor.Ball_Sensor_Activate();
  Set_Sensor.ENC_Activate();

  Halt_CAN_Data_Send = false;
}

void MainLoop(){
  //Set_LED.RED(HIGH);
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
  if(IPf10ms_count == 101) IPf10ms_count = 1;
}

void Interrupt_Processing_f1ms(){
  //frq = 1ms
  if(Halt_CAN_Data_Send == false){
    current_sum += adc_val_ch1[MOTOR_CURRENT];
  }

  //frq = 200ms
  if(IPf1ms_count % 200 == 0){
    HAL_CAN_Data_Output_ID0x1d2_466();
  }

  IPf1ms_count++;
  if(IPf1ms_count == 1001) IPf1ms_count = 1;
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

  if(Halt_CAN_Data_Send == false){
    if (current_val > 150){
      current = 1;
      Set_LED.GREEN(HIGH);
    } else Set_LED.GREEN(LOW);
    if (adc_val_ch1[BALL_SENSOR_VAL] < 100){
      photo = 1;
      Set_LED.BLUE(HIGH);
    } else Set_LED.BLUE(LOW);
      
    if (current == 1 && get_ball == 0){
      Set_LED.YELLOW(HIGH);
      get_ball = 1;
    } else if (photo == 0 && get_ball == 1){
      Set_LED.YELLOW(LOW);
      get_ball = 0;
    }

    printf("get ball : %d\n",get_ball);
    printf("current : %d,current val : %d\n",current,current_val);
    printf("photo   : %d,photo val   : %d\n",photo,adc_val_ch1[BALL_SENSOR_VAL]);

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

    current_sum = 0;
  }
}

void HAL_CAN_Data_Input_ID0x1d1_465(){
  Main_motor.ENABLE();
  Main_motor.Forward(canRecvData.data[0]);
  Set_LED.BLUE(HIGH);
}





