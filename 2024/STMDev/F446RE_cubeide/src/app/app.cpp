#include "app.h"
#include "main.h"
#include "Lib/DMAStream/DMAStream.h"
#include "Lib/CAN/CAN.hpp"
#include "i2c.h"

#define BNOAddress 0x28
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_EULER_H_LSB_ADDR 0x1A

CANBus can(&hcan1, 0x100);
CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {1, 2, 0, 0, 0, 0, 0, 0},
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
    }
}

void BNO055Check() {
    // BNO055が接続されているか確認
    // BNO055のチップIDを読み取り、0xA0であればBNO055が接続されていると判断
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    // Page 60 4.3.1 CHIP_ID
    uint8_t data;
    /**
     * HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
     * @brief  Read an amount of data in blocking mode from a specific memory address
     * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *                the configuration information for the specified I2C.
     * @param  DevAddress Target device address: The device 7 bits address value
     *         in datasheet must be shifted to the left before calling the interface
     * @param  MemAddress Internal memory address
     * @param  MemAddSize Size of internal memory address
     * @param  pData Pointer to data buffer
     * @param  Size Amount of data to be sent
     * @param  Timeout Timeout duration
     * @retval HAL status
     */
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress << 1, BNO055_CHIP_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (data != 0xA0) {
        printf("BNO055 not found\n");
    } else {
        printf("BNO055 found\n");
    }
}

void setup() {
}

void main_app() {
    setup();
    while (1) {
        BNO055Check();
        HAL_Delay(1000);
    }
}