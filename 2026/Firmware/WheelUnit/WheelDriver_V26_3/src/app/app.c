#include "app.h"

DigitalOut led1;
DigitalOut led2;
DigitalOut led3;

DigitalIn sw;

PwmOut ledr;
PwmOut ledg;
PwmOut ledb;

uint16_t adc_val[2];

SensoredVectorControl svc;

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

  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_val, 2);
  DigitalOut_Write(&led2, 1);

  // STSPIN32G4_Init(&hi2c3);

  BLDC_Init(&svc, true, &adc_val[0]);
  DigitalOut_Write(&led1, 1);

  printf("BLDC setup completed.\n");
}

void MainApp() {
  while (1) {
    double phase = 0;
    if (adc_val[0] > 1000) {
      DigitalOut_Write(&led1, 1);
    } else {
      DigitalOut_Write(&led1, 0);
    }
    // for (uint16_t i = 0; i < (TWO_PI * 10); i++) {
    //   phase += 0.2;
    //   BLDC_OpenLoopDrive(0.05, phase);
    //   HAL_Delay(1);
    // }
  }
}