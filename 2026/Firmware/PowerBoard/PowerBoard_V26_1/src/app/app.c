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

void Setup() {
  printf("PowerBoard Setup Start\n");

  PwmOut_Init(&led, &htim2, TIM_CHANNEL_4);

  Can_Init(&can_bus, &hcan, 0x100);

  DigitalIn_Init(&button, BUTTON_GPIO_Port, BUTTON_Pin);

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
        Kicker_Kick(1, 0.2);
        PwmOut_Write(&led, 0);
        break;
      case 0x14:  // チップキック
        Kicker_Kick(2, 0.2);
        PwmOut_Write(&led, 0);
        break;
      default:
        break;
    }
  }
}

void MainApp() {
  while (1) {
    if (Timer_ReadMs(&can_trasmit_interval_timer) > 10) {
      CanData can_send_data = {
          .stdId = 0x50,
          .data = {
              Kicker_DoneCheck()},
      };
      Can_Send(&can_bus, &can_send_data);
      Timer_Reset(&can_trasmit_interval_timer);
    }
  }
}
