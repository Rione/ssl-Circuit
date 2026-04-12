#include "app.h"

#include <stdint.h>
#include <stdio.h>

PwmOut kick1;
PwmOut kick2;
PwmOut led;

DigitalIn button;

DigitalOut lt_charge;
DigitalIn lt_done;

Serial serial;

CanBus can_bus;
CanData can_data;

void Setup() {
  printf("PowerBoard Setup Start\n");

  // PWMの初期化
  PwmOut_Init(&kick1, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&kick2, &htim2, TIM_CHANNEL_3);
  PwmOut_Init(&led, &htim2, TIM_CHANNEL_4);

  // CANの初期化
  CAN_Init(&can_bus, 0x100);

  // DigitalInOutの初期化
  DigitalIn_Init(&button, BUTTON_GPIO_Port, BUTTON_Pin);
  DigitalOut_Init(&lt_charge, LT_CHARGE_GPIO_Port, LT_CHARGE_Pin);
  DigitalIn_Init(&lt_done, LT_DONE_GPIO_Port, LT_DONE_Pin);

  printf("PowerBoard Setup Complete\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (Can_GetHandle(&can_bus) == hcan) {
    Can_Recv(&can_bus, &can_data);
    switch (can_data.stdId) {
      case 0x101:

        break;
      case 0x100:

        break;
      default:
        break;
    }
  }
}

void MainApp() {
  while (1) {
  }
}