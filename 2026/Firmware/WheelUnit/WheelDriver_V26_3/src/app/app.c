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

static void GetSensors() {
  encoder_val = adc_val[0];
  supply_volt_val = adc_val[1];

  supply_volt = supply_volt_val * ADC2VOLT * 20.0f;
  supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);

  temprature = adc_val[2] * ADC2VOLT * 100.0f - 50.0f;
}

// Packet format (11 bytes):
//   [0]    0xFF          header
//   [1]    command       0xFE=angular speed, 0xFC=torque/voltage
//   [2-3]  m[0] int16   motor 1 (big-endian)
//   [4-5]  m[1] int16   motor 2
//   [6-7]  m[2] int16   motor 3
//   [8-9]  m[3] int16   motor 4
//   [10]   0xAA          footer
static void RecvSerial() {
  const static uint8_t HEADER = 0xFF;
  const static uint8_t ANGULAR_SPEED_CMD = 0xFE;
  const static uint8_t TORQUE_CMD = 0xFC;
  const static uint8_t FOOTER = 0xAA;
  const static uint8_t PACKET_LEN = 11;
  static uint8_t recv_buf[11];
  static uint8_t index = 0;

  if (Serial_Available(&uart2)) {
    uint8_t recv_byte = Serial_Read(&uart2);

    if (index == 0) {
      if (recv_byte == HEADER) {
        recv_buf[0] = HEADER;
        index = 1;
      }
    } else if (index == PACKET_LEN - 1) {
      if (recv_byte == FOOTER) {
        uint32_t id = BLDC_GetId();
        uint8_t cmd = recv_buf[1];
        int16_t value = (int16_t)((recv_buf[2 + (id - 1) * 2] << 8) | recv_buf[2 + (id - 1) * 2 + 1]);
        if (cmd == 1) {
          mode = 1;
          target_angular_speed = value * 0.01f;
        } else {
          mode = 0;
          target_angular_speed = 0;
        }
        Timer_Reset(&serial_recv_timer);
      }
      index = 0;
    } else {
      recv_buf[index++] = recv_byte;
    }
  } else if (Timer_Read(&serial_recv_timer) > 0.5f) {
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
  DigitalOut_Write(&led3, 1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 3);
  HAL_Delay(100);
  DigitalOut_Write(&led2, 1);

  STSPIN32G4_Init(&hi2c3);
  HAL_Delay(100);

  BLDC_Init(false, 0, &adc_val[0]);

  Serial_Init(&uart2, &huart2, 256);

  LPF_Init(&supply_volt_lpf, 0.9, 12);

  Timer_Init(&serial_send_timer);
  Timer_Reset(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Reset(&serial_recv_timer);
  Timer_Init(&control_timer);

  printf("BLDC setup completed.\n");
}

void MainApp() {
  while (1) {
    GetSensors();
    SendSerial();

    if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT || is_voltage_out_of_range) {
      printf("Supply voltage out of range: %.2fV\n", supply_volt);
      is_voltage_out_of_range = true;
      BLDC_Stop(false);

      if (supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + 0.5f) && supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - 0.5f)) {
        is_voltage_out_of_range = false;
        PwmOut_Write(&ledr, 0);
      } else {
        PwmOut_Write(&ledr, 1);
        HAL_Delay(100);
        PwmOut_Write(&ledr, 0);
        HAL_Delay(100);
      }
    } else {
      RecvSerial();
      if (mode == 0) {
        BLDC_Stop(false);
      } else {
        if (mode == 1) {
          BLDC_AngularSpeedControl(target_angular_speed);
        } else if (mode == 2) {
          BLDC_VoltageControl(target_torque);
        }
      }
    }
    BLDC_SensoredVectorControlDrive(encoder_val, supply_volt);
  }
}
