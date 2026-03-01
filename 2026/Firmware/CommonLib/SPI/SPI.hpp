#ifndef __SPI_HPP__
#define __SPI_HPP__

#include "main.h"

#ifdef __cplusplus

class SPI {
 public:
  SPI(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort = nullptr, uint16_t csPin = 0);

  // 基本的な送受信
  uint8_t transfer(uint8_t data);
  void transfer(uint8_t* txData, uint8_t* rxData, uint16_t size);

  // 送信のみ
  void write(uint8_t data);
  void write(const uint8_t* data, uint16_t size);

  // 受信のみ
  uint8_t read();
  void read(uint8_t* data, uint16_t size);

  // チップセレクト制御
  void select();
  void deselect();

 private:
  SPI_HandleTypeDef* _hspi;
  GPIO_TypeDef* _csPort;
  uint16_t _csPin;
};

#endif
#endif
