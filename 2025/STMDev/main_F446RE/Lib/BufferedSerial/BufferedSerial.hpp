#ifndef __BUFFERED_SERIAL__
#define __BUFFERED_SERIAL__

#include "usart.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
// HAL_UART_Transmitの代わりに使うブロッキング送信 (理由はBufferedSerial.cpp参照)
void uartTransmitBlocking(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len, uint32_t timeout);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/*
 Initの順番をDMAが先になるようにする
 - MX_DMA_Init();
 - MX_USART2_UART_Init();
 - DMAはCircularモードにする
*/

class BufferedSerial {
  public:
    BufferedSerial(UART_HandleTypeDef *huart, uint16_t rxBufSize, uint16_t txBufSize = 64);
    void init(bool dma = false);
    bool available();
    uint8_t read();
    void flush();

    // DMAで送信する(送信完了を待たずに戻る)。前回の送信がまだ終わっていなければ今回分は破棄してfalseを返す。
    // 1パケットは1回のwriteで渡すこと(連続で呼ぶと2回目が前回の送信中に当たり破棄される)
    bool write(const uint8_t *data, uint16_t len);

    uint8_t getc() {
        return read();
    }

    void onRxError();
    bool matchesUart(UART_HandleTypeDef *huart) const;

    // 異常カウンタ
    volatile uint32_t rxErrorCount = 0;  // 受信エラー(ノイズ・フレーミング・オーバーラン等)の回数
    uint32_t txDropCount = 0;            // 前回の送信中で送れなかった回数

  private:
    void startRxDma();
    void restartRxDma();

    UART_HandleTypeDef *_huart;
    uint8_t *_rxBuf;
    uint16_t rxTop, rxBtm;
    uint16_t _rxBufSize;
    uint8_t *_txBuf;  // DMA送信中は呼び出し元のバッファが消えている可能性があるのでコピーして送る
    uint16_t _txBufSize;
    uint32_t _txStartTick;
    bool _useDMA;
    volatile bool _rxErrorPending;
};

#endif
#endif