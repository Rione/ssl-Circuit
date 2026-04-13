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
CanData can_recv_data;

Timer can_trasmit_interval_timer;

void Setup() {
  printf("PowerBoard Setup Start\n");

  // PWMの初期化
  PwmOut_Init(&kick1, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&kick2, &htim2, TIM_CHANNEL_3);
  PwmOut_Init(&led, &htim2, TIM_CHANNEL_4);

  // CANの初期化
  Can_Init(&can_bus, &hcan, 0x100);

  // DigitalInOutの初期化
  DigitalIn_Init(&button, BUTTON_GPIO_Port, BUTTON_Pin);
  DigitalOut_Init(&lt_charge, LT_CHARGE_GPIO_Port, LT_CHARGE_Pin);
  DigitalIn_Init(&lt_done, LT_DONE_GPIO_Port, LT_DONE_Pin);

  printf("PowerBoard Setup Complete\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (Can_GetHandle(&can_bus) == hcan) {
    Can_Recv(&can_bus, &can_recv_data);
    printf("CAN Received: ID=0x%X\n", can_recv_data.stdId);
    switch (can_recv_data.stdId) {
      case 0x11:  // 充電

        break;
      case 0x12:  // 放電

        break;
      case 0x13:  // ストレートキック

        break;
      case 0x14:  // チップキック

        break;
      default:
        break;
    }
  }
}

void MainApp() {
  while (1) {
    if (Timer_Read(&can_trasmit_interval_timer) > 10) {
      CanData can_send_data = {
          .stdId = 0x50,
          .data = {0},
      };
      Can_Send(&can_bus, &can_send_data);
      Timer_Reset(&can_trasmit_interval_timer);
    }
  }
}
