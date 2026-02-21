#include "kicker.hpp"

Kicker::Kicker(CANBus* can) { can_ = can; }

void Kicker::Kick(bool is_straight, uint8_t power, bool do_direct) {
  static Timer timer;
  if (timer.read_ms() < robot_params::kKickIntervalMs) return;

  CANBus::CANData canData = {
      .stdId = is_straight ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {power, (uint8_t)(do_direct ? 0xFF : 0), 0, 0, 0, 0, 0, 0},
  };
  can_->send(canData);

  // キックしたときの推定値を更新
  if (status_.cap_val_estimate >= power) {
    status_.cap_val_estimate -= power;
  } else {
    status_.cap_val_estimate = 0;
  }

  timer.reset();
}

void Kicker::Charge() {
  CANBus::CANData canData = {
      .stdId = can_id::kTxCharge,
      .data = {0},
  };
  can_->send(canData);
}

void Kicker::Discharge() {
  CANBus::CANData canData = {
      .stdId = can_id::kTxDischarge,
      .data = {0},
  };
  can_->send(canData);

  // 放電時はキャパシタ電圧推定値をリセット
  status_.cap_val_estimate = 0;
}

void Kicker::CancelDirect(bool is_straight) {
  CANBus::CANData canData = {
      .stdId = is_straight ? can_id::kTxStraightKick : can_id::kTxChipKick,
      .data = {0},
  };
  can_->send(canData);
}

void Kicker::UpdateCapValEstimate(uint8_t power) {
  if (status_.cap_val_estimate >= power) {
    status_.cap_val_estimate -= power;
  } else {
    status_.cap_val_estimate = 0;
  }
}