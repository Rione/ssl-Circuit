#ifndef __CAN_ID_HPP_
#define __CAN_ID_HPP_

#include <stdint.h>

namespace can_id {

// 送信用 CAN ID
constexpr uint32_t TX_CHARGE = 0x10;         // 充電指令
constexpr uint32_t TX_DISCHARGE = 0x11;      // 放電指令
constexpr uint32_t TX_STRAIGHT_KICK = 0x12;  // ストレートキック指令
constexpr uint32_t TX_CHIP_KICK = 0x13;      // チップキック指令
constexpr uint32_t TX_DRIBBLE = 0x14;        // ドリブル指令

// 受信用 CAN ID
constexpr uint32_t RX_VOLTAGE = 0x20;  // 電源情報
constexpr uint32_t RX_DRIBBLE = 0x21;  // ドリブラ情報

}  // namespace can_id

#endif  // __CAN_ID_HPP_
