#include "app.h"
#include "main.h"
#include "Lib/DMAStream/DMAStream.h"
#include "Lib/CAN/CAN.hpp"

int data = 0;
float fdata = 0.0;

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
    can.init();
    printfDMA("Init!!\n");
}

void main_app() {
    setup();
    while (1) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        printf("Toggle\n");
        can.send(canRecvData);
        HAL_Delay(1000);
    }
}