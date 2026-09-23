#include "SPI.hpp"

#ifdef __cplusplus

SPI::SPI(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : _hspi(hspi), _csPort(csPort), _csPin(csPin) {
  if (_csPort != nullptr) {
    deselect();  // 初期状態は非選択(High)
  }
}

uint8_t SPI::transfer(uint8_t data) {
  uint8_t rxData;
  HAL_SPI_TransmitReceive(_hspi, &data, &rxData, 1, HAL_MAX_DELAY);
  return rxData;
}

void SPI::transfer(uint8_t* txData, uint8_t* rxData, uint16_t size) {
  HAL_SPI_TransmitReceive(_hspi, txData, rxData, size, HAL_MAX_DELAY);
}

void SPI::write(uint8_t data) {
  HAL_SPI_Transmit(_hspi, &data, 1, HAL_MAX_DELAY);
}

void SPI::write(const uint8_t* data, uint16_t size) {
  HAL_SPI_Transmit(_hspi, (uint8_t*)data, size, HAL_MAX_DELAY);
}

uint8_t SPI::read() {
  uint8_t txData = 0xFF;  // ダミーデータ
  uint8_t rxData;
  HAL_SPI_TransmitReceive(_hspi, &txData, &rxData, 1, HAL_MAX_DELAY);
  return rxData;
}

void SPI::read(uint8_t* data, uint16_t size) {
  // 受信時はダミーデータを送信する必要があるためTransmitReceiveを使用するか、Receive単体を使うか検討する。
  // HAL_SPI_Receive は送信を行わないため、スレーブがクロックだけでデータを送る構成でない限り、通常はダミー送信が必要。
  // ここでは単純化のため HAL_SPI_Receive を使用するが、必要に応じてダミー送信を行う実装に変更可能。
  // ただし、多くのSPIデバイスはクロック供給のためにホストからの送信を必要とする。
  // HAL_SPI_Receive はFull-Duplexモードでも動作するが、送信データは未定義(0x00または前のデータ)になる可能性がある。
  // 安全のため、0xFFなどを送信しながら受信する実装にするのが一般的。

  // ここではシンプルに HAL_SPI_Receive を使用する。
  HAL_SPI_Receive(_hspi, data, size, HAL_MAX_DELAY);
}

void SPI::select() {
  if (_csPort != nullptr) {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
  }
}

void SPI::deselect() {
  if (_csPort != nullptr) {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
  }
}

#endif
