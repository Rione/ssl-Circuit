#ifndef __CAN_ID_HPP_
#define __CAN_ID_HPP_

#include <stdint.h>

namespace can_id {

// 送信用 CAN ID
constexpr uint16_t kTxOff = 0x10;           // ロボットの電源オフ
constexpr uint16_t kTxCharge = 0x11;        // 充電
constexpr uint16_t kTxDischarge = 0x12;     // 放電
constexpr uint16_t kTxStraightKick = 0x13;  // ストレートキック
constexpr uint16_t kTxChipKick = 0x14;      // チップキック

constexpr uint16_t kTxDribble = 0x20;  // ドリブル

// 受信用 CAN ID
constexpr uint16_t kRxSupplyBoard = 0x50;     // 電源・キッカー情報
constexpr uint16_t kRxChargeStart = 0x51;     // 充電開始通知
constexpr uint16_t kRxDischargeStart = 0x52;  // 放電開始通知

constexpr uint16_t kRxDribble = 0x70;  // ドリブル情報

}  // namespace can_id

#endif  // __CAN_ID_HPP_
