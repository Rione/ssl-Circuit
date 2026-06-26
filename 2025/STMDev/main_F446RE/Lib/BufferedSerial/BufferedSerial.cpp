#include "BufferedSerial.hpp"

namespace {

constexpr uint8_t kMaxBufferedSerials = 4;
BufferedSerial *g_serials[kMaxBufferedSerials] = {};
uint8_t g_serialCount = 0;

void registerSerial(BufferedSerial *serial) {
    for (uint8_t i = 0; i < g_serialCount; i++) {
        if (g_serials[i] == serial) {
            return;
        }
    }
    if (g_serialCount < kMaxBufferedSerials) {
        g_serials[g_serialCount++] = serial;
    }
}

BufferedSerial *findSerial(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < g_serialCount; i++) {
        if (g_serials[i] != nullptr && g_serials[i]->matchesUart(huart)) {
            return g_serials[i];
        }
    }
    return nullptr;
}

bool isRxRelatedError(uint32_t errorCode) {
    return (errorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE |
                         HAL_UART_ERROR_PE | HAL_UART_ERROR_DMA)) != 0U;
}

}  // namespace

BufferedSerial::BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize) 
    : _huart(huart),
     _rxBuf(new uint8_t[rxBufSize]),
     rxTop(0), rxBtm(0),
     _rxBufSize(rxBufSize),
     _useDMA(false),
     _rxErrorPending(false) {
}
    
void BufferedSerial::init(bool dma) {
    _useDMA = dma;
    registerSerial(this);
    startRxDma();
    printf("- Serial init\n");
}

void BufferedSerial::startRxDma() {
    rxBtm = 0;
    __HAL_UART_CLEAR_OREFLAG(_huart);
    __HAL_UART_CLEAR_FEFLAG(_huart);
    __HAL_UART_CLEAR_NEFLAG(_huart);
    _huart->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_DMA(_huart, _rxBuf, _rxBufSize);
}

void BufferedSerial::restartRxDma() {
    // HAL側でDMA受信は既に停止しているが、状態を揃えてから再開する
    (void)HAL_UART_AbortReceive(_huart);
    startRxDma();
    _rxErrorPending = false;
}

void BufferedSerial::onRxError() {
    _rxErrorPending = true;
}

bool BufferedSerial::matchesUart(UART_HandleTypeDef *huart) const {
    return _huart == huart;
}

bool BufferedSerial::available() {
    if (_rxErrorPending) {
        restartRxDma();
    }
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

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (!isRxRelatedError(huart->ErrorCode)) {
        return;
    }

    BufferedSerial *serial = findSerial(huart);
    if (serial != nullptr) {
        serial->onRxError();
    }
}
