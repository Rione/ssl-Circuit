#include "DMAStream.h"

static char dma_tx_buffer[USART2_TX_DMA_BUFFER_SIZE] = {0};

void printfDMA(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(dma_tx_buffer, USART2_TX_DMA_BUFFER_SIZE, format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_tx_buffer, strlen(dma_tx_buffer));
}