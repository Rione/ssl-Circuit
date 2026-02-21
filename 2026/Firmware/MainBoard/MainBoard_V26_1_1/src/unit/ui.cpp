#include "ui.hpp"
#include "robot.hpp"

UI::UI(BufferedSerial* serial) {
  serial_ = serial;
}

void UI::Recv(UiStatus& status) {
  if (serial_->available()) {
    const uint8_t kHeader = 0xFF;
    const uint8_t kFooter = 0xAA;
    const uint8_t kDataSize = 2;  // 受信バイト数
    static uint8_t recv_data[kDataSize];
    static uint8_t index = 0;
    uint8_t recv_byte;
    recv_byte = serial_->getc();

    if (index == 0) {
      if (recv_byte == kHeader) {  // ヘッダー確認
        index++;
      } else {
        index = 0;
      }
    } else if (index == (kDataSize + 1)) {
      if (recv_byte == kFooter) {  // フッター確認
        // 正常にデータを受信
        status.flags = recv_data[0];
        status.power_data = recv_data[1];
      }
      index = 0;
    } else {
      recv_data[index - 1] = recv_byte;
      index++;
    }
  }
}

void UI::Send(const Robot& robot) {
  const uint8_t kHeader = 0xFF;
  const uint8_t kFooter = 0xAA;
  const uint8_t kDataSize = 2;  // 送信バイト数
  uint8_t send_data[kDataSize + 2];

  send_data[0] = kHeader;
  send_data[1] = robot.info.battery_voltage;
  send_data[2] = robot.info.kicker_status.cap_val_estimate;
  send_data[3] = kFooter;

  serial_->write(send_data, sizeof(send_data));
}