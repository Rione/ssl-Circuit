#include "app.h"

#include <stdio.h>

#include "can_bus.h"
#include "can_id.h"
#include "main_mode.h"
#include "robot.h"

Robot robot;
CanData can_data;
MainMode main_mode;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (Can_GetHandle(&robot.can) == hcan) {
    Can_Recv(&robot.can, &can_data);
    switch (can_data.stdId) {
      case CAN_ID_RX_SUPPLY_BOARD:  // 電源・キッカー情報
        robot.info.kicker_status.done_charge = can_data.data[0];
        robot.info.battery_voltage = can_data.data[1] * 0.2;
        robot.info.kicker_status.cap_val = can_data.data[2];
        break;
      case CAN_ID_RX_DRIBBLER:  // ドリブル情報
        robot.info.dribble_status.data = can_data.data[0];
        break;
    }
  }
}

void Setup(void) {
  Robot_Initialize(&robot);
  MainMode_Init(&main_mode, &robot);
}

void MainApp(void) {
  while (1) {
    MainMode_Loop(&main_mode);
  }
}
