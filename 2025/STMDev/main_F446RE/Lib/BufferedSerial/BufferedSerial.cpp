#include "BufferedSerial.hpp"

#include <string.h>

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

BufferedSerial::BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize, uint16_t txBufSize)
    : _huart(huart),
     _rxBuf(new uint8_t[rxBufSize]),
     rxTop(0), rxBtm(0),
     _rxBufSize(rxBufSize),
     _txBuf(new uint8_t[txBufSize]),
     _txBufSize(txBufSize),
     _txStartTick(0),
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
    rxErrorCount++;
    _rxErrorPending = true;
}

bool BufferedSerial::write(const uint8_t *data, uint16_t len) {
    // ブロッキング送信だとUI(22byte@115200bps)で約1.9ms、Pi(13byte@230400bps)で約0.56ms
    // メインループが止まっていたので、DMAに任せてすぐ戻る
    if (_huart->gState != HAL_UART_STATE_READY) {
        if (HAL_GetTick() - _txStartTick < 100) {
            txDropCount++;
            return false;  // 前回の送信中
        }
        // 送信完了割り込みが来ずに送信中のまま止まった場合の復帰
        (void)HAL_UART_AbortTransmit(_huart);
    }
    if (len > _txBufSize) {
        len = _txBufSize;
    }
    memcpy(_txBuf, data, len);
    _txStartTick = HAL_GetTick();
    return HAL_UART_Transmit_DMA(_huart, _txBuf, len) == HAL_OK;
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

// HAL_UART_Transmitは送信残数をhuart->TxXferCountで数えるが、DMA受信中に受信エラー(ノイズ等)が
// 起きるとエラー割り込み(UART_DMAAbortOnError)がTxXferCountを0にしてしまう。送信ループの
// 「残数>0の確認」と「TxXferCount--」の間にこれが割り込むと0から減算されて65535に巻き戻り、
// 送信データを読み進めてRAM末尾を越えHardFaultする(UART4のNEで実際に発生)。
// 残数はローカル変数で数え、HALの状態変数に依存しない送信を行う。
extern "C" void uartTransmitBlocking(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len, uint32_t timeout) {
    uint32_t tickstart = HAL_GetTick();
    for (uint16_t i = 0; i < len; i++) {
        while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE)) {
            if (HAL_GetTick() - tickstart > timeout) return;
        }
        huart->Instance->DR = data[i];
    }
    while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TC)) {
        if (HAL_GetTick() - tickstart > timeout) return;
    }
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
