#ifndef __CAN_ID_HPP_
#define __CAN_ID_HPP_

#include <stdint.h>

namespace can_id {

// 送信用 CAN ID
constexpr uint32_t kTxCharge = 0x10;        // 充電指令
constexpr uint32_t kTxDischarge = 0x11;     // 放電指令
constexpr uint32_t kTxStraightKick = 0x12;  // ストレートキック指令
constexpr uint32_t kTxChipKick = 0x13;      // チップキック指令
constexpr uint32_t kTxDribble = 0x14;       // ドリブル指令

// 受信用 CAN ID
constexpr uint32_t kRxVoltage = 0x20;  // 電源情報
constexpr uint32_t kRxDribble = 0x21;  // ドリブラ情報

}  // namespace can_id

#endif  // __CAN_ID_HPP_
