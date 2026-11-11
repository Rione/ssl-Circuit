#include "app.h"

DigitalOut LED1;
DigitalOut LED2;
DigitalOut LED3;
DigitalOut LED4;

DigitalOut MD_SLEEP;

DigitalIn SW;

CanBus can;
CanData canRecvData;
static volatile uint8_t can_motor_level = 0;

Serial serial;

Timer can_send_interval_timer;

uint16_t adc_val[4];  // 0: Current, 1: BallSensor
#define MOTOR_CURRENT_IDX 0
#define BALL_SENSOR_IDX 1

#define CAN_SEND_INTERVAL_MS 10  // 10ms

#define CAN_RECV_ID 0x20
#define CAN_SEND_ID 0x70

#define ADC2_PA6_DIAGNOSTIC_MODE 0

// ADC2の順位: rank 1 = PA4 (MD_SO)、rank 2 = PA6 (BS_IN)
static void UpdateAdc2Inputs(void) {
  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
    adc_val[MOTOR_CURRENT_IDX] = HAL_ADC_GetValue(&hadc2);
  }
  if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
    adc_val[BALL_SENSOR_IDX] = HAL_ADC_GetValue(&hadc2);
  }
  HAL_ADC_Stop(&hadc2);
}

// センサ確認中は起動時キャリブレーションとモーター駆動を停止する
void Setup() {
  printf("Dribbler Setup Start\n");

  // F303K8T6ではLEDをGPIO出力として使用する
  DigitalOut_Init(&LED1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&LED2, LED2_GPIO_Port, LED2_Pin);
  DigitalOut_Init(&LED3, LED3_GPIO_Port, LED3_Pin);
  DigitalOut_Init(&LED4, LED4_GPIO_Port, LED4_Pin);

  // デジタル入出力の初期化
  DigitalOut_Init(&MD_SLEEP, MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin);

  DigitalOut_Write(&MD_SLEEP, 1);

  DigitalOut_Write(&LED1, 1);  // デジタル入出力の初期化確認

  // CANの初期化
  Can_Init(&can, &hcan1, 0);
  HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  DigitalOut_Write(&LED2, 1);  // CANの初期化確認

  // ADCの初期化
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  printf("ADC2 PA4/PA6 start\n");

  DigitalOut_Write(&LED3, 1);  // ADCの初期化確認

  // モーターの初期化
  Dribbler_Init();
  while (!Motor_SetBaseCurrent(adc_val[MOTOR_CURRENT_IDX])) {
    UpdateAdc2Inputs();
    HAL_Delay(1);
  }
  while (!Dribbler_SetPhotoThreshold(adc_val[BALL_SENSOR_IDX])) {
    UpdateAdc2Inputs();
    HAL_Delay(1);
  }
  printf("Motor Base Current Set Complete\n");

  // キャリブレーション完了後は1秒待ってからセンサ判定を開始する。
  HAL_Delay(1000);
  DigitalOut_Write(&BS_LED, 1);

  DigitalOut_Write(&LED4, 1);  // モーターの初期化確認

  // タイマーの初期化
  Timer_Init(&can_send_interval_timer);

  DigitalOut_Write(&LED1, 0);
  DigitalOut_Write(&LED2, 0);
  DigitalOut_Write(&LED3, 0);
  // キャリブレーション完了後、USER_LED4へHIGHを出力
  DigitalOut_Write(&LED4, 1);

  printf("Dribbler Setup Complete\n");
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void MainApp() {
  // 前回送信した状態（変化検出用）。初回は必ず送信されるように-1で初期化
  static int16_t prev_by_photo = -1;
  static int16_t prev_by_current = -1;
  static int16_t prev_captured = -1;
  uint32_t last_debug_ms = 0;
  uint32_t last_loop_debug_ms = 0;
  while (1) {
    UpdateAdc2Inputs();
    // BS_LEDは常時点灯させる
    DigitalOut_Write(&BS_LED, 1);

    Dribbler_Update(adc_val[BALL_SENSOR_IDX], adc_val[MOTOR_CURRENT_IDX]);

    uint8_t by_photo = Dribbler_IsBallCapturedByPhoto();
    uint8_t by_current = Dribbler_IsBallCapturedByCurrent();
    uint8_t captured = Dribbler_IsBallCaptured();

    // UART1へBS_INの値を10回/秒で出力する。
    if (HAL_GetTick() - last_debug_ms >= 100) {
      printf("current_filtered=%u photo_adc=%u adc2_state=%lu adc2_error=0x%08lX\r\n",
             Motor_GetFilteredCurrent(), adc_val[BALL_SENSOR_IDX],
             (unsigned long)HAL_ADC_GetState(&hadc2),
             (unsigned long)HAL_ADC_GetError(&hadc2));
      last_debug_ms = HAL_GetTick();
    }

    if (HAL_GetTick() - last_loop_debug_ms >= 1000) {
      printf("MAIN_LOOP\r\n");
      last_loop_debug_ms = HAL_GetTick();
    }

    // CANから速度指令を受けている間はCANを優先する。
    // 指令0のときだけ、遮光で逆転するローカル動作を使う。
    if (can_motor_level > 0) {
      Motor_Drive(can_motor_level);
    } else if (by_photo) {
      Motor_DriveReverse(MAX_SPEED_LEVEL);
    } else {
      Motor_Free();
    }

    // いずれかの値が変化した瞬間、または10ms周期でCAN送信
    bool changed = (by_photo != prev_by_photo) ||
                   (by_current != prev_by_current) ||
                   (captured != prev_captured);
    bool interval_elapsed =
        Timer_ReadMs(&can_send_interval_timer) >= CAN_SEND_INTERVAL_MS;

    if (changed || interval_elapsed) {
      CanData data = {
          .stdId = CAN_SEND_ID,
          .data =
              {
                  by_photo,
                  by_current,
                  captured,
              },
      };
      Can_Send(&can, &data);
      Timer_Reset(&can_send_interval_timer);

      prev_by_photo = by_photo;
      prev_by_current = by_current;
      prev_captured = captured;
    }

    DigitalOut_Write(&LED1, Dribbler_IsBallCapturedByPhoto());
    DigitalOut_Write(&LED2, Dribbler_IsBallCapturedByCurrent());
    DigitalOut_Write(&LED3, Dribbler_IsBallCaptured());
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (can.hcan == hcan) {
    Can_Recv(&can, &canRecvData);
    if (canRecvData.stdId == CAN_RECV_ID) {
      // 受信した0~255の速度データをレベルに変換する。
      // 実際のPWM出力は、競合しないようMainApp内で行う。
      can_motor_level = (canRecvData.data[0] * MAX_SPEED_LEVEL) / 255;
    }
  }
}
