#include "robot_link.h"

#include "config.h"

void RobotLink::begin() {
  UiProto_ParserInit(&parser_);
  Serial1.setTX(PIN_UART_TX);
  Serial1.setRX(PIN_UART_RX);
  Serial1.setFIFOSize(UART_RX_FIFO);
  Serial1.begin(UART_BAUD);
}

void RobotLink::update(const TestCommand &cmd) {
  while (Serial1.available()) {
    uint8_t type, len;
    const uint8_t *payload;
    if (UiProto_Parse(&parser_, (uint8_t)Serial1.read(), &type, &payload, &len)) {
      handleFrame(type, payload, len);
    }
  }

  uint32_t now = millis();
  if (now - last_tx_ms_ >= COMMAND_PERIOD_MS) {
    last_tx_ms_ = now;
    sendCommand(cmd);
  }
}

void RobotLink::handleFrame(uint8_t type, const uint8_t *p, uint8_t len) {
  if (type != UI_PROTO_TYPE_TELEMETRY || len != UI_PROTO_TELEMETRY_LEN) return;

  auto s16 = [&](uint8_t ofs) { return (int16_t)UiProto_GetU16(&p[ofs]); };

  tlm_.battery_v = UiProto_GetU16(&p[UI_TLM_BATTERY]) * 0.01f;
  tlm_.cap_v = UiProto_GetU16(&p[UI_TLM_CAP]);
  tlm_.flags1 = p[UI_TLM_FLAGS1];
  tlm_.flags2 = p[UI_TLM_FLAGS2];
  for (int i = 0; i < 4; i++) {
    tlm_.wheel_radps[i] = s16(UI_TLM_WHEEL + i * 2) * 0.01f;
  }
  tlm_.accel_x_g = s16(UI_TLM_ACCEL_X) * 0.001f;
  tlm_.accel_y_g = s16(UI_TLM_ACCEL_Y) * 0.001f;
  tlm_.yaw_rate_dps = s16(UI_TLM_YAW_RATE) * 0.1f;
  tlm_.yaw_deg = s16(UI_TLM_YAW) * 0.01f;
  tlm_.dribble_pct = p[UI_TLM_DRIBBLE];
  tlm_.kick_ack = p[UI_TLM_KICK_ACK];

  has_rx_ = true;
  last_rx_ms_ = millis();
  rx_frames_++;
}

void RobotLink::sendCommand(const TestCommand &cmd) {
  uint8_t payload[UI_PROTO_COMMAND_LEN];
  uint8_t flags = 0;
  if (cmd.test_enable) {
    flags |= UI_CMD_F_TEST_ENABLE;
    if (cmd.dribble_on) flags |= UI_CMD_F_DRIBBLE_ON;
    if (cmd.charge) flags |= UI_CMD_F_CHARGE;
    if (cmd.wheel_run) flags |= UI_CMD_F_WHEEL_RUN;
  }
  payload[UI_CMD_FLAGS] = flags;
  payload[UI_CMD_DRIBBLE] = cmd.dribble_pct;
  payload[UI_CMD_KICK_POWER] = cmd.kick_pct;
  payload[UI_CMD_KICK_SEQ] = cmd.kick_seq;
  payload[UI_CMD_KICK_TYPE] = cmd.kick_type;
  payload[UI_CMD_WHEEL_MASK] = cmd.wheel_mask & 0x0F;
  float target = constrain(cmd.wheel_target_radps, -WHEEL_TEST_MAX_RADPS, WHEEL_TEST_MAX_RADPS);
  UiProto_PutU16(&payload[UI_CMD_WHEEL_TARGET], (uint16_t)(int16_t)lroundf(target * 100.0f));

  uint8_t frame[UI_PROTO_COMMAND_LEN + UI_PROTO_OVERHEAD];
  uint16_t n = UiProto_Build(frame, UI_PROTO_TYPE_COMMAND, payload, UI_PROTO_COMMAND_LEN);
  Serial1.write(frame, n);
}
