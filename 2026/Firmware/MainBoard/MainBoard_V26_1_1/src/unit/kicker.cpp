#include "kicker.hpp"

Kicker::Kicker(CANBus* can) { can_ = can; }

void Kicker::Kick(bool type, uint8_t power, bool do_direct) {
  static Timer timer;
  if (timer.read_ms() < robot_params::kKickIntervalMs) return;

  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {power, (uint8_t)(do_direct ? 0xFF : 0), 0, 0, 0, 0, 0, 0},
  };
  can_->send(canData);

  // キックしたときの推定値を更新
  if (cap_val_estimate_ >= power) {
    cap_val_estimate_ -= power;
  } else {
    cap_val_estimate_ = 0;
  }

  timer.reset();
}

void Kicker::ChargeControl(bool type) {
  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxCharge : can_id::kTxDischarge,
      .data = {0},
  };
  can_->send(canData);

  // 放電開始時に推定値をリセット
  if (type == can_id::kRxDischargeStart) {
    cap_val_estimate_ = 0;
  }
}

void Kicker::StopDirect(bool type) {
  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {0},
  };
  can_->send(canData);
}
