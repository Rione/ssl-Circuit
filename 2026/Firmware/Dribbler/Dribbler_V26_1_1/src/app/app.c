#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;

DigitalOut CAN_LED;
DigitalOut BS_LED;
DigitalOut MD_SLEEP;

DigitalIn SW;

// Serial pc;

uint16_t adc_val[2];  // ADCの値を格納する配列

float current;

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

      // ADCの初期化
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 2);
      for (uint8_t i = 0; i < sizeof(adc_val); i++) {
            while (!(adc_val[i] > 0));  // ADCの値が代入されるまで待つ
      }
      printf("ADC_DMA start\n");

      Motor_Init();
}

void GetSensors() {
}

void MainApp() {
}