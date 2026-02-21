#include "app.h"

#include "main_mode.hpp"
#include "robot.hpp"

Robot robot;
CANBus::CANData canData;
MainMode mainMode(&robot);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (robot.can.getHcan() == hcan) {
    robot.can.recv(canData);
    switch (canData.stdId) {
      case can_id::kRxSupplyBoard:  // 電源・キッカー情報
        robot.info.kicker_status.do_direct_status = canData.data[0];
        break;
      case can_id::kRxChargeStart:  // 充電開始通知
        break;
      case can_id::kRxDischargeStart:  // 放電開始通知
        robot.kicker.SetCapValEstimate(0);
        break;
      case can_id::kRxStraightKick:  // ストレートキック通知
        robot.kicker.UpdateCapValEstimate(canData.data[0]);
        break;
      case can_id::kRxChipKick:  // チップキック通知
        robot.kicker.UpdateCapValEstimate(canData.data[0]);
        break;
      case can_id::kRxDribbler:  // ドリブル情報

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