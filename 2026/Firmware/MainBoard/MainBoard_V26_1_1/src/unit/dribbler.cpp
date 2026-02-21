#include "dribbler.hpp"

Dribbler::Dribbler(CANBus* can) : can_(can) {}

void Dribbler::Send(uint8_t power, bool force_send) {
  static Timer timer;
  static uint8_t dribble_power_prev = 0;

  // 前回と同じパワーかつ、強制送信フラグが立っていない場合
  if (power == dribble_power_prev && !force_send) {
    // 100ms経過していないなら送信しない（CANバス負荷軽減）
    if (timer.read_ms() < 100) {
      return;
    }
  }

  // 現状の仕様では0以外は最大出力(100)にする
  uint8_t send_power = (power != 0) ? 100 : 0;

  CANBus::CANData canData = {
      .stdId = can_id::kTxDribbler,
      .data = {send_power, 0, 0, 0, 0, 0, 0, 0},
  };
  can_->send(canData);

  timer.reset();
  dribble_power_prev = power;
}
