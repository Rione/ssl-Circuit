#include "ui.hpp"

UI::UI(BufferedSerial* serial) {
      serial_ = serial;
}

void UI::Recv() {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 2;  // 受信データサイズ(バイト数)
      static uint8_t recv_data[data_size];
      static uint8_t index = 0;
      uint8_t recv_byte;

      if (serial_->available()) {
            recv_byte = serial_->getc();

            if (index == 0) {
                  if (recv_byte == HEADER) {  // ヘッダー確認
                        index++;
                  } else {
                        index = 0;
                  }
            } else if (index == (data_size + 1)) {
                  if (recv_byte == FOOTER) {  // フッター確認
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
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 2;
      uint8_t send_data[data_size];

      send_data[0] = HEADER;
      send_data[1] = FOOTER;

      serial_->write(send_data, data_size);
}