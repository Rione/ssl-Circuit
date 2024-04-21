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
    BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize)
        : _huart(huart),
          _rxBuf(new uint8_t[rxBufSize]),
          rxTop(0), rxBtm(0),
          _rxBufSize(rxBufSize),
          _useDMA(false) {
    }
    void init(bool dma = false) {
        _useDMA = dma;
        HAL_UART_Receive_DMA(_huart, _rxBuf, _rxBufSize);
        printf("- Serial init\n");
    }

    bool available() {
        uint16_t rxTop = _rxBufSize - _huart->hdmarx->Instance->NDTR;
        return rxTop != rxBtm;
    }

    uint8_t read() {
        uint16_t rxTop = _rxBufSize - _huart->hdmarx->Instance->NDTR;
        if (rxTop == rxBtm) {
            return 0;
        }
        uint8_t data = _rxBuf[rxBtm];
        rxBtm = (rxBtm + 1) % _rxBufSize;
        // printf("top:%d btm:%d NDTR:%d data:%d\n", rxTop, rxBtm, _huart->hdmarx->Instance->NDTR, data);
        return data;
    }

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