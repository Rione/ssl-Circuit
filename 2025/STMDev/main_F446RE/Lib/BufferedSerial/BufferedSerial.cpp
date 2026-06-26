#include "BufferedSerial.hpp"

BufferedSerial::BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize) 
    : _huart(huart),
     _rxBuf(new uint8_t[rxBufSize]),
     rxTop(0), rxBtm(0),
     _rxBufSize(rxBufSize),
     _useDMA(false) {
}
    
void BufferedSerial::init(bool dma) {
    _useDMA = dma;
    HAL_UART_Receive_DMA(_huart, _rxBuf, _rxBufSize);
    printf("- Serial init\n");
}

bool BufferedSerial::available() {
    uint16_t rxTop = _rxBufSize - _huart->hdmarx->Instance->NDTR;
    return rxTop != rxBtm;
}

uint8_t BufferedSerial::read() {
    uint16_t rxTop = _rxBufSize - _huart->hdmarx->Instance->NDTR;
    if (rxTop == rxBtm) {
        return 0;
    }
    uint8_t data = _rxBuf[rxBtm];
    rxBtm = (rxBtm + 1) % _rxBufSize;
    // printf("top:%d btm:%d NDTR:%d data:%d\n", rxTop, rxBtm, _huart->hdmarx->Instance->NDTR, data);
    return data;
}

void BufferedSerial::flush() {
    // 読み取り位置をDMAの現在位置まで進め、未読データを破棄する
    rxBtm = _rxBufSize - _huart->hdmarx->Instance->NDTR;
}


