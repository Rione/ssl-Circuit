#ifndef __DMA_Stream__
#define __DMA_Stream__

#include "main.h"
#include "dma.h"
#include "usart.h"

#include "stdio.h"
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define USART2_TX_DMA_BUFFER_SIZE 128

asm(".global _printf_float");

void printfDMA(const char *format, ...);
#endif