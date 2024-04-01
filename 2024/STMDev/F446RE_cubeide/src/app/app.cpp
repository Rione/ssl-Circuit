#include "app.h"
#include "main.h"

int data = 0;
float fdata = 0.0;

void main_app() {
    // Your code here
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // HAL_Delay(250); // Delay 500ms
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(250); // Delay 500ms
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    data++;
    fdata += 0.1;

    // // ここでDMAバッファにprintfの出力をコピー
    // snprintf(dma_buffer, DMA_BUFFER_SIZE, "Hello, world!\r\n");
    // // DMA転送開始
    // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_buffer, strlen(dma_buffer));
    // DMA転送完了まで待機
    // printfDMA("Hello, world! %d %.2f\r\n", data, fdata);
}