#include "KickerBoard.hpp"

KickerBoard::KickerBoard(CANBus* canBus) : _canBus(canBus) {
}

void KickerBoard::init() {
}

void KickerBoard::kick(uint8_t type, uint8_t power, uint8_t doDirect) {
  static Timer timer;
  if (timer.read_ms() < 500) {
    return;
  } else if (timer.read_ms() > 500) {
    timer.set_ms(500);
  }
  type = (type == 0) ? KICK_STRAIGHT : KICK_CHIP;  // type : 0: straight, 1: chip
  doDirect = (doDirect == 1) ? 0xFF : 0;

  CANBus::CANData canData = {
      .stdId = type,
      .data = {power, doDirect, 0, 0, 0, 0, 0, 0},
  };

  capValEstimate -= power;  // キックした分だけ充電量を減らす
  if (capValEstimate < 0) {
    capValEstimate = 0;
  }

  _canBus->send(canData);

  timer.reset();
}

void KickerBoard::resetDoDirect(uint8_t type) {
  type = (type == 0) ? KICK_STRAIGHT : KICK_CHIP;  // type : 0: straight, 1: chip
  CANBus::CANData canData = {
      .stdId = type,
      .data = {0, 0, 0, 0, 0, 0, 0, 0xFF},
  };
  _canBus->send(canData);
}

void KickerBoard::chargeControl(uint8_t state) {
  static Timer timer;
  if (timer.read_ms() < 100) {
    return;
  } else if (timer.read_ms() > 100) {
    timer.set_ms(100);
  }

  state = (state == 0) ? CHARGE_START : DISCHARGE_START;  // 0: charge, 1: discharge
  CANBus::CANData canData = {
      .stdId = state,
      .data = {0},
  };

  if (state == DISCHARGE_START) {
    capValEstimate = 0;  // dischargeしたら充電量を0にする
  }

  _canBus->send(canData);

  timer.reset();
}
