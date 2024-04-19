#include "app.h"
#include "main.h"
#include "DMAStream.h"
#include "CAN.hpp"
#include "Timer.hpp"
#include "i2c.h"
#include "BNO055.hpp"
#include "DigitalInOut.hpp"

CANBus can(&hcan1, 0x100);
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);
DigitalOut led1(LED1_GPIO_Port, LED1_Pin);
CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {1, 2, 0, 0, 0, 0, 0, 0},
};

BNO055 bno(&hi2c1);
Timer timer;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
    }
}

void setup() {
    printf("Start %d %d\n", (BNOAddress + 1) << 1, ((BNOAddress) << 1) + 1);
    for (size_t i = 0; i < 10; i++) {
        bno.check();
    }
    bno.setUnit(1, 1, 1, 0);
    bno.setPowerMode();
    bno.setOperaitonMode(OPERATION_MODE_AMG);
    bno.accConfig();
    bno.init();
}

void main_app() {
    setup();
    while (1) {
        timer.reset();
        acc_t acc = bno.getAcc();
        printfDMA("Acc: %.2f %.2f %.2f time:%ld\n", acc.x, acc.y, acc.z, timer.read_us());
        HAL_Delay(20);
        led1 = !led1;
        // HAL_Delay(100);
        // printfDMA("Hello\n");
    }
}