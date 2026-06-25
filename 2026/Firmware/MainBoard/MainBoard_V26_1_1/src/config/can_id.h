#ifndef __CAN_ID_H_
#define __CAN_ID_H_

#include <stdint.h>

// 送信用 CAN ID
#define CAN_ID_TX_OFF           ((uint16_t)0x10)  // ロボットの電源オフ
#define CAN_ID_TX_CHARGE        ((uint16_t)0x11)  // 充電
#define CAN_ID_TX_DISCHARGE     ((uint16_t)0x12)  // 放電
#define CAN_ID_TX_STRAIGHT_KICK ((uint16_t)0x13)  // ストレートキック
#define CAN_ID_TX_CHIP_KICK     ((uint16_t)0x14)  // チップキック

#define CAN_ID_TX_DRIBBLER ((uint16_t)0x20)  // ドリブル

// 受信用 CAN ID
#define CAN_ID_RX_SUPPLY_BOARD    ((uint16_t)0x50)  // 電源・キッカー情報
#define CAN_ID_RX_CHARGE_START    ((uint16_t)0x51)  // 充電開始通知
#define CAN_ID_RX_DISCHARGE_START ((uint16_t)0x52)  // 放電開始通知
#define CAN_ID_RX_STRAIGHT_KICK   ((uint16_t)0x53)  // ストレートキック通知
#define CAN_ID_RX_CHIP_KICK       ((uint16_t)0x54)  // チップキック通知

#define CAN_ID_RX_DRIBBLER ((uint16_t)0x70)  // ドリブル情報

#endif  // __CAN_ID_H_
