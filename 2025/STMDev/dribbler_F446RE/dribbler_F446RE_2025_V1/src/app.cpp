#include "app.hpp"
#include "adc.h"
#include "math.h"

uint16_t sw_val = 0;
uint16_t adc_val_ch1[4];

uint32_t IPf10ms_count = 1;
uint32_t IPf1ms_count = 1;

int current_sum[10] = {0,0,0,0,0,0,0,0,0,0};
int current_sum_count = 0;
int Motor_Ajust_Value;
uint8_t get_ball = 0;
int speed = 0;

CANBus can = CANBus(&hcan1, 0);
CANBus::CANData canRecvData;
bool Halt_Get_Current;
bool Halt_CAN_Data_Send;
bool Halt_CAN_Data_Output_ID0x1d2_245;
uint8_t Change_Motor_Speed = 0;

Basic_IO_Control_Extension_Sensor Set_Sensor;
Basic_IO_Control_Motor Main_motor;
Basic_IO_Control_LED Set_LED;
Basic_IO_Control_LED_Flash IPf100ms_Flash;
Basic_IO_Control_LED_Flash IPf200ms_Flash;
Basic_IO_Control_LED_Flash IPf300ms_Flash;
Basic_IO_Control_LED_Flash IPf500ms_Flash;

AD_Setup_Control AD_Setup;

void Setup(void){
  Halt_CAN_Data_Output_ID0x1d2_245 = true;
  Halt_Get_Current = true;
  Halt_CAN_Data_Send = true;

  HAL_Delay(500);
  Set_LED.ALL_Control(HIGH);
  HAL_Delay(1000);
  Set_LED.ALL_Control(LOW);
  HAL_Delay(500);

  IPf100ms_Flash.LED_Flash_Activate = true;
  IPf200ms_Flash.LED_Flash_Activate = true;
  IPf200ms_Flash.LED_Flash_CAN = START;
  AD_Setup.ADC_Check();
  AD_Setup.Administrator_Privilege();
  AD_Setup.DRV_Check();
  AD_Setup.Motor_Check();
  HAL_Delay(500);
  IPf200ms_Flash.LED_Flash_CAN = STOP;
  IPf200ms_Flash.LED_Flash_Activate = false;
  IPf100ms_Flash.LED_Flash_Activate = false;

  Main_motor.Brake();

  can.init();
  Set_Sensor.Ball_Sensor_Activate();
  Set_Sensor.ENC_Activate();

  Halt_CAN_Data_Output_ID0x1d2_245 = false;
  Halt_Get_Current = false;

  HAL_Delay(1000);
  Set_LED.ALL_Control_EX_CAN(LOW);

  Halt_CAN_Data_Send = false;

  IPf500ms_Flash.LED_Flash_Activate = true;
  IPf500ms_Flash.LED_Flash_RED = START;
}

void MainLoop(){
  
}

void Interrupt_Processing_f10ms(){
  //frq = 10ms
  //Control User Switch 
  sw_val = HAL_GPIO_ReadPin(USER_SW_GPIO_Port,USER_SW_Pin);

  //frq = 100ms
  if(IPf10ms_count % 10 == 0){
    IPf100ms_Flash.LED_Flash_Control();

    if(Change_Motor_Speed != 0){
      Change_Motor_Speed++;
    }
    if(Change_Motor_Speed > 2){
      Halt_Get_Current = false;
    }
    if(Change_Motor_Speed > 4){
      Change_Motor_Speed = 0;
    }
  }

  //frq = 200ms
  if(IPf10ms_count % 20 == 0){
    IPf200ms_Flash.LED_Flash_Control();
  }

  //frq = 300ms
  if(IPf10ms_count % 30 == 0){
    IPf300ms_Flash.LED_Flash_Control();
  }

  //frq = 500ms
  if(IPf10ms_count % 50 == 0){
    IPf500ms_Flash.LED_Flash_Control();
  }

  IPf10ms_count++;
  if(IPf10ms_count == 101) IPf10ms_count = 1;
}

void Interrupt_Processing_f1ms(){
  //frq = 1ms
  if(Halt_Get_Current == false){
    current_sum[0] += adc_val_ch1[MOTOR_CURRENT];
    current_sum_count++;
    if(current_sum_count == 10){
      if(Halt_CAN_Data_Output_ID0x1d2_245 == false){
        CAN_Data_Output_ID0x1d2_245();
      }
      current_sum_count = 0;
      current_sum[0] = 0;
    }
  }

  IPf1ms_count++;
  if(IPf1ms_count == 1001) IPf1ms_count = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (can.getHcan() == hcan){
    can.recv(canRecvData);
    switch (canRecvData.stdId){
    case 0xf4: //244
      CAN_Data_Input_ID0x1d1_244();
      break;
    default:
      break;
    }
  }
}

void CAN_Data_Output_ID0x1d2_245(){
  uint16_t current_val = 0;
  uint8_t photo = 0;
  uint8_t current = 0;

  for(int i = 0;i < 10;i++){
    current_val += current_sum[i];
  }
  for(int i = 9;i > 0;i--){
    current_sum[i] = current_sum[i - 1];
  }
  current_sum[0] = 0;
  current_val /= 100;

  if(Halt_CAN_Data_Send == false){
    Set_LED.CAN_LED(HIGH);

    if (current_val > Motor_Base_Current +  MOTOR_CURRENT_THRESHOLD + Motor_Ajust_Value){
      current = 1;
      Set_LED.GREEN(HIGH);
      if(Change_Motor_Speed != 0) Set_LED.GREEN(LOW);
    } else Set_LED.GREEN(LOW);
    if (adc_val_ch1[BALL_SENSOR_VAL] < PHOTO_THRESHOLD){
      photo = 1;
      Set_LED.BLUE(HIGH);
    } else Set_LED.BLUE(LOW);

    if(Change_Motor_Speed == 0){
      if(photo == 1){
        if(current == 1 || (current == 0 && get_ball == 1)){
          get_ball = 1;
          Set_LED.YELLOW(HIGH);
        } else {
          get_ball = 0;
          Set_LED.YELLOW(LOW);
        }
      } else {
        get_ball = 0;
        Set_LED.YELLOW(LOW);
      }
    } else {
      get_ball = 0;
      current_val = 0;
      Set_LED.YELLOW(LOW);
    }
    
    CANBus::CANData data = {
      .stdId = 0xf5,
      .data = {
        get_ball,
        current,
        (uint8_t)(current_val & 0xFF),
        (uint8_t)((current_val >> 8) & 0xFF),
        photo,
        (uint8_t)(adc_val_ch1[BALL_SENSOR_VAL] & 0xFF),
        (uint8_t)((adc_val_ch1[BALL_SENSOR_VAL] >> 8) & 0xFF),
      },
    };
    can.send(data);
    printf("ball:%d__I:%d,val=%3d__P:%d,val=%3d__ajv=%3d\n",get_ball,current,current_val,photo,adc_val_ch1[BALL_SENSOR_VAL],Motor_Ajust_Value);
  } else {
    Set_LED.CAN_LED(LOW);
  }
}

void CAN_Data_Input_ID0x1d1_244(){
  Main_motor.ENABLE();
  int data = canRecvData.data[0];
  if(data > 0){
    Main_motor.Forward(data);
  } else if(data == 0){
    Main_motor.Brake();
  }
  if(speed != data){
    Halt_Get_Current = true;
    Change_Motor_Speed = 1;
  }
  speed = data;
}





