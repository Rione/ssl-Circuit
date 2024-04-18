#ifndef __DMA_Stream__
#define __DMA_Stream__

#include "main.h"
#include "dma.h"
#include "usart.h"

#include "stdio.h"
#include <string.h>
#include <stdarg.h>

#define USE_USART1_DISABLE_USART2

asm(".global _printf_float");

#ifdef USE_USART1_DISABLE_USART2

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define USART1_TX_DMA_BUFFER_SIZE 128
static char dma_tx_buffer[USART1_TX_DMA_BUFFER_SIZE] = {0};

inline void printfDMA(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(dma_tx_buffer, USART1_TX_DMA_BUFFER_SIZE, format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dma_tx_buffer, strlen(dma_tx_buffer));
}
#endif

#ifndef USE_USART1_DISABLE_USART2

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define USART2_TX_DMA_BUFFER_SIZE 128
static char dma_tx_buffer[USART2_TX_DMA_BUFFER_SIZE] = {0};

inline void printfDMA(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(dma_tx_buffer, USART2_TX_DMA_BUFFER_SIZE, format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_tx_buffer, strlen(dma_tx_buffer));
}
#endif

#endif