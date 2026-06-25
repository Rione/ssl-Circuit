#include "app.h"

#include "can_bus.h"
#include "can_id.h"
#include "main_mode.h"
#include "robot.h"

Robot   robot;
CanData can_data;
MainMode main_mode;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (Can_GetHandle(&robot.can) == hcan) {
    Can_Recv(&robot.can, &can_data);
    switch (can_data.stdId) {
      case CAN_ID_RX_SUPPLY_BOARD:  // 電源・キッカー情報
        robot.info.kicker_status.do_direct_status = can_data.data[0];
        break;
      case CAN_ID_RX_CHARGE_START:  // 充電開始通知
        break;
      case CAN_ID_RX_DISCHARGE_START:  // 放電開始通知
        Kicker_SetCapValEstimate(&robot.kicker, 0);
        break;
      case CAN_ID_RX_STRAIGHT_KICK:  // ストレートキック通知
        Kicker_UpdateCapValEstimate(&robot.kicker, can_data.data[0]);
        break;
      case CAN_ID_RX_CHIP_KICK:  // チップキック通知
        Kicker_UpdateCapValEstimate(&robot.kicker, can_data.data[0]);
        break;
      case CAN_ID_RX_DRIBBLER:  // ドリブル情報
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
