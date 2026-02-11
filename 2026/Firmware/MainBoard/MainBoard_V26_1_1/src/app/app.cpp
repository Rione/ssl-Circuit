#include "app.h"

#include "main_mode.hpp"
#include "robot.hpp"

Robot robot;
CANBus::CANData canData;
MainMode mainMode(&robot);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (robot.can.getHcan() == hcan) {
    robot.can.recv(canData);
    switch (canData.stdId) {
    case can_id::kRxSupplyBoard: // 電源・キッカー情報

      break;
    case can_id::kRxChargeStart: // 充電開始通知

      break;
    case can_id::kRxDischargeStart: // 放電開始通知

      break;
    case can_id::kRxDribble: // ドリブル情報

      break;
    }
  }
}

void Setup() { robot.Initialize(); }

void MainApp() {
  while (1) {
    mainMode.Loop();
  }
}