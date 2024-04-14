#include "app.h"
#include "main.h"
#include "Lib/DMAStream/DMAStream.h"
#include "Lib/CAN/CAN.hpp"

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

void setup() {
}

void main_app() {
    setup();
    while (1) {
        HAL_I2C_Master_Transmit(&hi2c1, 0x50 << 1, (uint8_t *)"Hello", 5, 1000);
        HAL_Delay(1000);
    }
}