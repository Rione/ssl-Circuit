#include "app.h"

#include <stdint.h>
#include <stdio.h>

#include "kicker.h"

PwmOut led;

DigitalIn button;

Serial serial;

CanBus can_bus;
CanData can_recv_data;

Timer can_trasmit_interval_timer;

uint16_t adc_value[3];
// adc_value[0]: 電源電圧 (PB0/CH11, 分圧 10k/1k)
// adc_value[1]: 昇圧電圧 (PB1/CH12, 分圧 1M/4.7k)
// adc_value[2]: 内部温度センサ

// ADC -> 実値換算
#define ADC_VREF 3.3f          // ADC基準電圧 [V]
#define ADC_MAX 4095.0f        // 12bit
#define SUPPLY_DIVIDER 11.0f   // (10k + 1k) / 1k
#define BOOST_DIVIDER 213.77f  // (1M + 4.7k) / 4.7k
#define TEMP_V25 1.43f         // 温度センサ 25degC時の出力 [V]
#define TEMP_SLOPE 0.0043f     // 温度センサ傾き 4.3mV/degC

static inline float Adc_ToVoltage(uint16_t raw) {
  return (float)raw / ADC_MAX * ADC_VREF;
}

float GetSupplyVoltage() {  // 電源電圧 [V]
  return Adc_ToVoltage(adc_value[0]) * SUPPLY_DIVIDER;
}

float GetBoostVoltage() {  // 昇圧電圧 [V]
  return Adc_ToVoltage(adc_value[1]) * BOOST_DIVIDER;
}

float GetTemperature() {  // 内部温度 [degC] (概算)
  return (TEMP_V25 - Adc_ToVoltage(adc_value[2])) / TEMP_SLOPE + 25.0f;
}

void Setup() {
  printf("PowerBoard Setup Start\n");

  PwmOut_Init(&led, &htim2, TIM_CHANNEL_4);

  Can_Init(&can_bus, &hcan, 0x100);

  DigitalIn_Init(&button, BUTTON_GPIO_Port, BUTTON_Pin);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 3);

  Kicker_Init();

  Timer_Init(&can_trasmit_interval_timer);

  printf("PowerBoard Setup Complete\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (Can_GetHandle(&can_bus) == hcan) {
    Can_Recv(&can_bus, &can_recv_data);
    switch (can_recv_data.stdId) {
      case 0x11:  // 充電
        Kicker_Charge();
        PwmOut_Write(&led, 1);
        break;
      case 0x12:  // 放電
        Kicker_Discharge();
        PwmOut_Write(&led, 0);
        break;
      case 0x13:  // ストレートキック
        Kicker_Kick(1, can_recv_data.data[0] / 255.0f);
        PwmOut_Write(&led, 0);
        break;
      case 0x14:  // チップキック
        Kicker_Kick(2, can_recv_data.data[0] / 255.0f);
        PwmOut_Write(&led, 0);
        break;
      default:
        break;
    }
  }
}

void MainApp() {
  while (1) {
    Kicker_SetBoostVoltage(GetBoostVoltage());  // 最新の昇圧電圧を渡す
    Kicker_Update();                            // キックパルスの終了処理(非ブロッキング)

    if (Timer_ReadMs(&can_trasmit_interval_timer) > 10) {
      CanData can_send_data = {
          .stdId = 0x50,
          .data = {
              Kicker_DoneCheck(),
              GetSupplyVoltage() * 5,  // 電源電圧 [V] * 10
              GetBoostVoltage(),       // 昇圧電圧 [V]
          },
      };
      Can_Send(&can_bus, &can_send_data);
      Timer_Reset(&can_trasmit_interval_timer);
    }
  }
}