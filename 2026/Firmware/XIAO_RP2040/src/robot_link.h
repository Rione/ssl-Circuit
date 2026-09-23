// MainBoard との UART 通信 (プロトコルは include/ui_protocol.h)
#pragma once

#include <Arduino.h>

#include "config.h"
#include "ui_protocol.h"

struct Telemetry {
  float battery_v = 0;
  float cap_v = 0;
  uint8_t flags1 = 0;
  uint8_t flags2 = 0;
  float wheel_radps[4] = {0, 0, 0, 0};
  float accel_x_g = 0;
  float accel_y_g = 0;
  float yaw_rate_dps = 0;
  float yaw_deg = 0;
  uint8_t dribble_pct = 0;
  uint8_t kick_ack = 0;

  bool has(uint8_t flag1) const { return flags1 & flag1; }
};

struct TestCommand {
  bool test_enable = false;
  bool dribble_on = false;
  bool charge = false;
  bool wheel_run = false;
  uint8_t dribble_pct = 0;
  uint8_t kick_pct = 0;
  uint8_t kick_seq = 0;
  uint8_t kick_type = UI_KICK_NONE;
  uint8_t wheel_mask = 0;
  float wheel_target_radps = 0;
};

class RobotLink {
 public:
  void begin();
  // 受信処理と周期送信。loop から頻繁に呼ぶ
  void update(const TestCommand &cmd);

  bool connected() const { return has_rx_ && millis() - last_rx_ms_ < LINK_LOST_MS; }
  const Telemetry &telemetry() const { return tlm_; }
  uint32_t rxFrames() const { return rx_frames_; }

 private:
  void handleFrame(uint8_t type, const uint8_t *payload, uint8_t len);
  void sendCommand(const TestCommand &cmd);

  UiProtoParser parser_{};
  Telemetry tlm_;
  bool has_rx_ = false;
  uint32_t last_rx_ms_ = 0;
  uint32_t last_tx_ms_ = 0;
  uint32_t rx_frames_ = 0;
};
