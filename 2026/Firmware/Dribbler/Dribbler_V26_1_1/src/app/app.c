#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;

DigitalOut CAN_LED;
DigitalOut MD_SLEEP;

DigitalIn SW;

CanBus can;
CanData canRecvData;

Serial serial;

Timer can_send_interval_timer;
Timer control_cycle_timer;

uint16_t adc_val[4];  // 0: Current, 1: BallSensor
#define MOTOR_CURRENT_IDX 0
#define BALL_SENSOR_IDX 1

#define CONTROL_FREQ_HZ 1000      // 1000Hz
#define CONTROL_INTERVAL_US 1000  // 1000us
#define CAN_SEND_INTERVAL_MS 1    // 10ms

#define CAN_RECV_ID 0x20
#define CAN_SEND_ID 0x70

void Setup() {
  printf("Dribbler Setup Start\n");

  // PWMの初期化
  PwmOut_Init(&LED1, &htim8, TIM_CHANNEL_4);
  PwmOut_Init(&LED2, &htim8, TIM_CHANNEL_3);
  PwmOut_Init(&LED3, &htim8, TIM_CHANNEL_2);
  PwmOut_Init(&LED4, &htim8, TIM_CHANNEL_1);

  // デジタル入出力の初期化
  DigitalOut_Init(&CAN_LED, CAN_LED_GPIO_Port, CAN_LED_Pin);
  DigitalOut_Init(&MD_SLEEP, MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin);

  DigitalOut_Write(&MD_SLEEP, 1);

  PwmOut_Write(&LED1, 1);  // デジタル入出力の初期化確認

  // CANの初期化
  Can_Init(&can, &hcan1, 0);
  HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  PwmOut_Write(&LED2, 1);  // CANの初期化確認

  // ADCの初期化
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 4);
  printf("ADC_DMA start\n");

  PwmOut_Write(&LED3, 1);  // ADCの初期化確認

  // モーターの初期化
  Dribbler_Init();
  while (!Motor_SetBaseCurrent(adc_val[MOTOR_CURRENT_IDX])) {
    HAL_Delay(1);
  }
  while (!Dribbler_SetPhotoThreshold(adc_val[BALL_SENSOR_IDX])) {
    HAL_Delay(1);
  }
  printf("Motor Base Current Set Complete\n");

  PwmOut_Write(&LED4, 1);  // モーターの初期化確認

  // タイマーの初期化
  Timer_Init(&can_send_interval_timer);
  Timer_Init(&control_cycle_timer);

  PwmOut_Write(&LED1, 0);
  PwmOut_Write(&LED2, 0);
  PwmOut_Write(&LED3, 0);
  PwmOut_Write(&LED4, 0);

  printf("Dribbler Setup Complete\n");
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void MainApp() {
  while (1) {
    Dribbler_Update(adc_val[BALL_SENSOR_IDX], adc_val[MOTOR_CURRENT_IDX]);

    if (Timer_ReadMs(&can_send_interval_timer) >= CAN_SEND_INTERVAL_MS) {
      DigitalOut_Write(&CAN_LED, 1);
      CanData data = {
          .stdId = CAN_SEND_ID,
          .data =
              {
                  Dribbler_IsBallCapturedByPhoto(),
                  Dribbler_IsBallCapturedByCurrent(),
                  Dribbler_IsBallCaptured(),
              },
      };
      Can_Send(&can, &data);
      Timer_Reset(&can_send_interval_timer);
    } else {
      DigitalOut_Write(&CAN_LED, 0);
    }

    PwmOut_Write(&LED1, Dribbler_IsBallCapturedByPhoto());
    PwmOut_Write(&LED2, Dribbler_IsBallCapturedByCurrent());
    PwmOut_Write(&LED3, Dribbler_IsBallCaptured());

    while (Timer_ReadUs(&control_cycle_timer) <= CONTROL_INTERVAL_US);
    Timer_Reset(&control_cycle_timer);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (can.hcan == hcan) {
    Can_Recv(&can, &canRecvData);
    if (canRecvData.stdId == CAN_RECV_ID) {
      // 受信した0~255の速度データをレベルに変換
      uint8_t level = (canRecvData.data[0] * MAX_SPEED_LEVEL) / 255;
      Motor_Drive(level);
    }
  }
}