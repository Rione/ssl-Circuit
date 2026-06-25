#include "app.h"

#define ADC2VOLT 0.0008058608059f

DigitalOut led1;
DigitalOut led2;
DigitalOut led3;

DigitalIn sw;

PwmOut ledr;
PwmOut ledg;
PwmOut ledb;

Serial uart2;

LPF supply_volt_lpf;

uint16_t adc_val[3];

uint16_t encoder_val;
uint16_t supply_volt_val;
float supply_volt;
float temprature;

bool is_voltage_out_of_range;

float target_angular_speed, target_torque;

uint8_t mode = 0;

Timer serial_send_timer;
Timer serial_recv_timer;
Timer control_timer;

// USART2でORE等の受信エラーが発生した場合、DMA異常停止からの復帰を待たずに即座に受信を再開する。
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART2) {
    Serial_Reset(&uart2);
  }
}

static void GetSensors() {
  encoder_val = adc_val[0];
  supply_volt_val = adc_val[1];

  supply_volt = supply_volt_val * ADC2VOLT * 20.0f;
  supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);

  temprature = adc_val[2] * ADC2VOLT * 100.0f - 50.0f;
}

static void RecvSerial() {
  const static uint8_t HEADER = 0xFF;
  const static uint8_t FOOTER = 0xAA;
  const static uint8_t PACKET_LEN = 9;
  static uint8_t recv_buf[9];
  static uint8_t index = 0;

  while (Serial_Available(&uart2)) {
    uint8_t recv_byte = Serial_Read(&uart2);

    if (index == 0) {
      if (recv_byte == HEADER) {
        index++;
      } else {
        index = 0;
      }
    } else if (index == (PACKET_LEN + 1)) {
      if (recv_byte == FOOTER) {
        uint32_t id = BLDC_GetId();
        uint8_t cmd = recv_buf[0];
        int16_t value = (int16_t)((recv_buf[1 + (id - 1) * 2] << 8) | recv_buf[1 + (id - 1) * 2 + 1]);
        if (cmd == 1) {
          mode = 1;
          target_angular_speed = value * 0.01f;
        } else {
          mode = 0;
          target_angular_speed = 0;
        }
        Timer_Reset(&serial_recv_timer);
        DigitalOut_Write(&led1, 1);
      }
      index = 0;
    } else {
      recv_buf[index - 1] = recv_byte;
      index++;
    }
  }
  if (Timer_Read(&serial_recv_timer) > 1.0f) {
    DigitalOut_Write(&led1, 0);
    mode = 0;
    Serial_Reset(&uart2);
    Timer_Reset(&serial_recv_timer);
  }
}

static void SendSerial() {
  if (Timer_ReadUs(&serial_send_timer) > SERIAL_SEND_INTERVAL_US) {
    const static uint8_t HEADER = 0xFF;
    const static uint8_t FOOTER = 0xAA;
    static uint8_t data[5];

    data[0] = HEADER;
    data[1] = (is_voltage_out_of_range << 1) | (mode != 0);
    data[2] = ((int16_t)(BLDC_GetAngularSpeed() * 100) >> 8) & 0xFF;
    data[3] = (int16_t)(BLDC_GetAngularSpeed() * 100) & 0xFF;
    data[4] = FOOTER;

    Serial_Write(&uart2, data, sizeof(data));
    Timer_Reset(&serial_send_timer);
  }
}

void Setup() {
  printf("BLDC setup started.\n");

  DigitalOut_Init(&led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&led2, LED2_GPIO_Port, LED2_Pin);
  DigitalOut_Init(&led3, LED3_GPIO_Port, LED3_Pin);

  DigitalIn_Init(&sw, SW_GPIO_Port, SW_Pin);

  PwmOut_Init(&ledr, &htim4, TIM_CHANNEL_2);
  PwmOut_Init(&ledg, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&ledb, &htim3, TIM_CHANNEL_1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 3);
  HAL_Delay(10);
  DigitalOut_Write(&led1, 1);

  STSPIN32G4_Init(&hi2c3);
  DigitalOut_Write(&led2, 1);
  HAL_Delay(10);

  DigitalOut_Write(&led3, 1);
  BLDC_Init(false, 0, &adc_val[0]);

  Serial_Init(&uart2, &huart2, 512);

  LPF_Init(&supply_volt_lpf, 0.9, 12);

  Timer_Init(&serial_send_timer);
  Timer_Reset(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Reset(&serial_recv_timer);
  Timer_Init(&control_timer);
  Timer_Reset(&control_timer);

  DigitalOut_Write(&led1, 0);
  DigitalOut_Write(&led2, 0);
  DigitalOut_Write(&led3, 0);

  printf("BLDC setup completed.\n");
}

// モーター電源(VM)が後から投入された場合に STSPIN32G4 を設定する。
// 電源(READY)と電圧が安定してから一度だけ設定し、リセット連発を防ぐ。
// フォルト(NFAULT=0)時は STATUS レジスタを読んで種別を表示する(診断用)。
// 戻り値: ゲートドライバが設定済みで駆動可能なら true
static bool UpdateGateDriver() {
  static bool configured = false;
  static uint32_t last_action_ms = 0;
  static uint32_t not_ready_since_ms = 0;

  bool ready = STSPIN32G4_IsReady();
  bool fault = STSPIN32G4_IsFault();
  bool power_ok = (supply_volt > SUPPLY_VOLTAGE_MIN_LIMIT && supply_volt < SUPPLY_VOLTAGE_MAX_LIMIT);

  // 電源が落ちた/電圧異常 → 未設定状態に戻し、次に電源が揃ったら再設定。
  // ただしSTSPIN32G4_Configure()内のリセットコマンド直後はREADYピンが
  // 一瞬だけ不安定になることがあるため、即座に未設定状態へ戻すと
  // 「再設定→READYが乱れる→再設定」の無限ループに陥る。
  // 100ms以上continuously not-readyが続いた場合のみ未設定状態に戻す。
  if (!ready || !power_ok) {
    if (not_ready_since_ms == 0) {
      not_ready_since_ms = HAL_GetTick();
    } else if (HAL_GetTick() - not_ready_since_ms > 100) {
      configured = false;
    }
    return false;
  }
  not_ready_since_ms = 0;

  if (!configured) {
    // 電源が揃った直後の設定(連続実行を防ぐため500ms間隔)
    if (HAL_GetTick() - last_action_ms > 500) {
      HAL_Delay(20);  // 電源安定待ち

      // ★ブートストラップ・コンデンサ充電★
      // コールド起動時はハイサイド駆動用のブートストラップが空で、
      // いきなり通常PWMを出すとVDS_P(過電流)保護が誤作動してラッチする。
      // 全相ローサイドONにして先にブートストラップを充電してからクリアする。
      BLDC_ForceLowSide();         // 全相ローサイドON(ハイサイドOFF)
      STSPIN32G4_Configure();      // reset + buck + nfault
      STSPIN32G4_ClearRegister();  // フォルトクリア→ブリッジ有効、ローサイド導通
      HAL_Delay(80);               // ブートストラップ充電(Vds低のためフォルト出ない)
      STSPIN32G4_ClearRegister();  // 残フォルトクリア
      HAL_Delay(2);

      if (!STSPIN32G4_IsFault()) {
        configured = true;
        printf("STSPIN32G4 configured OK. (supply=%.2fV)\n", supply_volt);
      } else {
        printf("STSPIN32G4 still FAULT after config (supply=%.2fV): ", supply_volt);
        STSPIN32G4_ReadStatus();  // どのフォルトか表示
      }
      last_action_ms = HAL_GetTick();
    }
  } else if (fault) {
    // 設定後に発生したフォルトを表示(500ms間隔)
    if (HAL_GetTick() - last_action_ms > 500) {
      printf("STSPIN32G4 FAULT during run (supply=%.2fV): ", supply_volt);
      STSPIN32G4_ReadStatus();
      last_action_ms = HAL_GetTick();
    }
  }

  return configured;
}

void MainApp() {
  while (1) {
    GetSensors();
    bool driver_ready = UpdateGateDriver();
    SendSerial();
    RecvSerial();

    if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT || is_voltage_out_of_range) {
      printf("Supply voltage out of range: %.2fV\n", supply_volt);
      is_voltage_out_of_range = true;
      BLDC_Stop();

      if (supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + 0.5f) && supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - 0.5f)) {
        is_voltage_out_of_range = false;
        DigitalOut_Write(&led3, 0);
      } else {
        DigitalOut_Write(&led3, 1);
        HAL_Delay(100);
        DigitalOut_Write(&led3, 0);
        HAL_Delay(100);
      }
    } else if (!driver_ready) {
      // ゲートドライバ準備完了前は駆動しない(積分項も積み上げない)
      BLDC_Stop();
      DigitalOut_Write(&led2, 1);
    } else {
      DigitalOut_Write(&led2, 0);
      if (mode == 0) {
        BLDC_Stop();
      } else if (mode == 1) {
        BLDC_AngularSpeedControl(target_angular_speed);
        PwmOut_Write(&ledr, Abs(BLDC_GetAmpVolt() * 0.1));
      }
      BLDC_SensoredVectorControlDrive(encoder_val, supply_volt);
    }

    DigitalOut_Write(&led3, 1);
    while (Timer_ReadUs(&control_timer) < CONTROL_INTERVAL_US);
    DigitalOut_Write(&led3, 0);
    Timer_Reset(&control_timer);
  }
}
