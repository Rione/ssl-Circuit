#include "kicker.hpp"

Kicker::Kicker(CANBus* can) { can_ = can; }

void Kicker::Kick(bool type, uint8_t power, bool do_direct) {
  if (kick_interval_timer_.read_ms() < robot_params::kKickIntervalMs)
    return;

  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {power, (uint8_t)(do_direct ? 0xFF : 0), 0, 0, 0, 0, 0, 0},
  };
  can_->send(canData);

  kick_interval_timer_.reset();
}

void Kicker::ChargeControl(bool type) {
  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxCharge : can_id::kTxDischarge,
      .data = {0},
  };
  can_->send(canData);
}

void Kicker::StopDirect(bool type) {
  CANBus::CANData canData = {
      .stdId = type ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {0},
  };
  can_->send(canData);
}
