#include "app.h"

DigitalOut led1;
DigitalOut led2;
DigitalOut led3;

DigitalIn sw;

PwmOut ledr;
PwmOut ledg;
PwmOut ledb;

uint16_t adc_val[2];

Timer control_timer;

float debug1;
float debug2;

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
  HAL_Delay(100);
  DigitalOut_Write(&led2, 1);

  STSPIN32G4_Init(&hi2c3);
  HAL_Delay(100);

  BLDC_Init(true, &adc_val[0]);

  Timer_Init(&control_timer);

  printf("BLDC setup completed.\n");
}

void MainApp() {
  uint32_t last_toggle_ms = HAL_GetTick();
  bool reverse = false;

  while (1) {
    uint16_t encoder_value = adc_val[0];
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_toggle_ms >= 1000U) {
      last_toggle_ms = now_ms;
      reverse = !reverse;
    }

    double target_torque = reverse ? -20.0 : 20.0;

    // 1秒ごとに正転と逆転を切り替える
    // BLDC_VoltageControl(0.2);
    BLDC_AngularSpeedControl(50);
    // BLDC_PositionControl(PI);
    BLDC_SensoredVectorControlDrive(encoder_value, 20);  // エンコーダ値と電圧を渡して駆動

    debug1 = BLDC_GetMechTheta();
    debug2 = BLDC_GetAngularSpeed();
    while (Timer_ReadUs(&control_timer) < 100) {
    }
    Timer_Reset(&control_timer);
  }
}