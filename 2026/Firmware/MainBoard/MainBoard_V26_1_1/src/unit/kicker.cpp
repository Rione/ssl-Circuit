#include "kicker.hpp"

Kicker::Kicker(CANBus* can) {
      _can = can;
}

void Kicker::Kick(bool type, uint8_t power, bool do_direct) {
      if (_kick_interval_timer.read_ms() < params::KICK_INTERVAL_MS) {
            return;
      }

      CANBus::CANData canData = {
          .stdId = type ? can_id::TX_STRAIGHT_KICK : can_id::TX_CHIP_KICK,
          .data = {power, (do_direct ? 0xFF : 0), 0, 0, 0, 0, 0, 0},
      };
      _can->send(canData);

      _kick_interval_timer.reset();
}

void Kicker::ChargeControl(bool type) {
      CANBus::CANData canData = {
          .stdId = type ? can_id::TX_CHARGE : can_id::TX_DISCHARGE,
          .data = {0},
      };
      _can->send(canData);
}

void Kicker::StopDirect(bool type) {
      CANBus::CANData canData = {
          .stdId = type ? can_id::TX_STRAIGHT_KICK : can_id::TX_CHIP_KICK,
          .data = {0},
      };
      _can->send(canData);
}
