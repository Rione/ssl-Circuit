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
        // battery_voltageはMainBoard自身のローカルADC(Robot_UpdateSensor)を正とするため、
        // ここでは上書きしない
        robot.info.kicker_status.cap_val = can_data.data[2];
        break;
      case CAN_ID_RX_DRIBBLER:  // ドリブル情報
        robot.info.dribble_status.data = can_data.data[0];
        break;
    }
  }
}

// DMA受信のエラー (DMA転送エラー等) で HAL が受信を中止したときに呼ばれる。
// 何もしないとその系統は二度と受信しなくなるので、受信だけを再開する
// (Serial_Reset は送信DMAも止めるので使わない。md_serials[2] はWheelUnitへの指令送信にも使う)。
// ※ USART の割り込みは有効にしていないため、フレーミングエラー等ではこれは呼ばれない。
//    受信の途絶は OmniDrive_Recv の受信監視でも検出して再開する
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  for (int i = 0; i < 4; i++) {
    if (robot.md_serials[i].huart == huart) {
      Serial_RestartRx(&robot.md_serials[i]);
      robot.omni_drive.wheel_rx_restart_count[i]++;
      return;
    }
  }
  if (robot.serial4.huart == huart) {
    Serial_RestartRx(&robot.serial4);
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
