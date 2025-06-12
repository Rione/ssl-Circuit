#ifndef __BUFFERED_SERIAL__
#define __BUFFERED_SERIAL__

#include "usart.h"
#include <stdio.h>

#ifdef __cplusplus

/*
 Initの順番をDMAが先になるようにする
 - MX_DMA_Init();
 - MX_USART2_UART_Init();
 - DMAはCircularモードにする
*/

class BufferedSerial {
  public:
    BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize);
    void init(bool dma = false);
    bool available();
    uint8_t read();

    void write(uint8_t data) {
        HAL_UART_Transmit(_huart, &data, 1, 100);
    }

    void write(const uint8_t *data, uint16_t len) {
        HAL_UART_Transmit(_huart, (uint8_t *)data, len, 100);
    }

    uint8_t getc() {
        return read();
    }

  private:
    UART_HandleTypeDef *_huart;
    uint8_t *_rxBuf;
    uint16_t rxTop, rxBtm;
    uint16_t _rxBufSize;
    bool _useDMA;
};

#endif
#endif