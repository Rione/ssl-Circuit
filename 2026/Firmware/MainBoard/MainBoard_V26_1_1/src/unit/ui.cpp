#include "ui.hpp"

UI::UI(BufferedSerial* serial) {
  serial_ = serial;
}

void UI::Recv() {
  const uint8_t kHeader = 0xFF;
  const uint8_t kFooter = 0xAA;
  const uint8_t kDataSize = 2;  // 受信データサイズ(バイト数)
  static uint8_t recv_data[kDataSize];
  static uint8_t index = 0;
  uint8_t recv_byte;

  if (serial_->available()) {
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
      }
      index = 0;
    } else {
      recv_data[index - 1] = recv_byte;
      index++;
    }
  }
}

void UI::Send() {
  const uint8_t kHeader = 0xFF;
  const uint8_t kFooter = 0xAA;
  const uint8_t kDataSize = 2;
  uint8_t send_data[kDataSize];

  send_data[0] = kHeader;
  send_data[1] = kFooter;

  serial_->write(send_data, kDataSize);
}