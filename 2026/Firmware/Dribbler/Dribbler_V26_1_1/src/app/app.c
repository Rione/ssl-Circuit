#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;

DigitalOut CAN_LED;
DigitalOut BS_LED;
DigitalOut MD_SLEEP;

DigitalIn SW;

CanBus can;
CanData canRecvData;

Serial serial;

uint16_t adc_val[2];  // 0: Current, 1: BallSensor

#define MOTOR_CURRENT_IDX 0
#define BALL_SENSOR_IDX 1

// Thresholds
#define MOTOR_CURRENT_THRESHOLD 500
#define PHOTO_THRESHOLD 2000
#define MOTOR_BASE_CURRENT 0

// Global variables
uint32_t current_sum[10] = {0};
int current_sum_count = 0;
int speed = 0;
uint8_t get_ball = 0;
int Motor_Ajust_Value = 0;

bool Halt_Get_Current = false;
bool Halt_CAN_Data_Send = false;
uint8_t Change_Motor_Speed = 0;

// Function Prototypes
void CAN_Data_Output_ID0x1d2_246();
void CAN_Data_Input_ID0x1d1_245();

void Setup() {
      printf("Dribbler Setup Start\n");

      // PWMの初期化
      PwmOut_Init(&LED1, &htim8, TIM_CHANNEL_4);
      PwmOut_Init(&LED2, &htim8, TIM_CHANNEL_3);
      PwmOut_Init(&LED3, &htim8, TIM_CHANNEL_2);
      PwmOut_Init(&LED4, &htim8, TIM_CHANNEL_1);

      // デジタル入出力の初期化
      DigitalOut_Init(&CAN_LED, CAN_LED_GPIO_Port, CAN_LED_Pin);
      DigitalOut_Init(&BS_LED, BS_LED_GPIO_Port, BS_LED_Pin);
      DigitalOut_Init(&MD_SLEEP, MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin);

      DigitalOut_Write(&MD_SLEEP, 1);
      DigitalOut_Write(&BS_LED, 1);

      // ADCの初期化
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 2);

      printf("ADC_DMA start\n");

      Motor_Init();
      Motor_Brake();

      Can_Init(&can, &hcan1, 0);
}

void MainApp() {
      uint32_t tick_count = 0;

      while (1) {
            HAL_Delay(1);

            // 1ms logic
            if (!Halt_Get_Current) {
                  current_sum[0] += adc_val[MOTOR_CURRENT_IDX];
                  current_sum_count++;
                  if (current_sum_count >= 10) {
                        CAN_Data_Output_ID0x1d2_246();
                        current_sum_count = 0;
                  }
            }

            // 10ms logic
            if (tick_count % 10 == 0) {
                  if (Change_Motor_Speed != 0) {
                        Change_Motor_Speed++;
                        if (Change_Motor_Speed > 2) {
                              Halt_Get_Current = false;
                        }
                        if (Change_Motor_Speed > 4) {
                              Change_Motor_Speed = 0;
                        }
                  }
            }
            tick_count++;
      }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
      if (can.hcan == hcan) {
            Can_Recv(&can, &canRecvData);
            if (canRecvData.stdId == 0xf4) {
                  CAN_Data_Input_ID0x1d1_245();
            }
      }
}

void CAN_Data_Input_ID0x1d1_245() {
      int data = canRecvData.data[0];

      if (data > 0) {
            Motor_Drive(data / 255.0f);
      } else {
            Motor_Brake();
      }

      if (speed != data) {
            Halt_Get_Current = true;
            Change_Motor_Speed = 1;
            speed = data;
      }
}

void CAN_Data_Output_ID0x1d2_246() {
      uint32_t current_val = 0;
      uint8_t photo = 0;
      uint8_t current_detected = 0;

      for (int i = 0; i < 10; i++) {
            current_val += current_sum[i];
      }
      for (int i = 9; i > 0; i--) {
            current_sum[i] = current_sum[i - 1];
      }
      current_sum[0] = 0;

      current_val /= 100;

      if (!Halt_CAN_Data_Send) {
            DigitalOut_Write(&CAN_LED, 1);

            if (current_val > (uint32_t)(MOTOR_BASE_CURRENT + MOTOR_CURRENT_THRESHOLD + Motor_Ajust_Value)) {
                  current_detected = 1;
                  PwmOut_Write(&LED1, 1.0f);
                  if (Change_Motor_Speed != 0) PwmOut_Write(&LED1, 0.0f);
            } else {
                  PwmOut_Write(&LED1, 0.0f);
            }

            if (adc_val[BALL_SENSOR_IDX] < PHOTO_THRESHOLD) {
                  photo = 1;
                  PwmOut_Write(&LED2, 1.0f);
            } else {
                  PwmOut_Write(&LED2, 0.0f);
            }

            if (Change_Motor_Speed == 0) {
                  if (photo == 1) {
                        if (current_detected == 1 || (current_detected == 0 && get_ball == 1)) {
                              get_ball = 1;
                              PwmOut_Write(&LED3, 1.0f);
                        } else {
                              get_ball = 0;
                              PwmOut_Write(&LED3, 0.0f);
                        }
                  } else {
                        get_ball = 0;
                        PwmOut_Write(&LED3, 0.0f);
                  }
            } else {
                  get_ball = 0;
                  current_val = 0;
                  PwmOut_Write(&LED3, 0.0f);
            }

            CanData data = {
                .stdId = 0xf5,
                .data = {
                    get_ball,
                    current_detected,
                    (uint8_t)(current_val & 0xFF),
                    (uint8_t)((current_val >> 8) & 0xFF),
                    photo,
                    (uint8_t)(adc_val[BALL_SENSOR_IDX] & 0xFF),
                    (uint8_t)((adc_val[BALL_SENSOR_IDX] >> 8) & 0xFF),
                    0,
                },
            };
            Can_Send(&can, &data);
      } else {
            DigitalOut_Write(&CAN_LED, 0);
      }
}