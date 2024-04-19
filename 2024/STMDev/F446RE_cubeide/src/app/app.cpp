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
DigitalOut led2(LED2_GPIO_Port, LED2_Pin);

CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {100, 200, 0, 0, 0, 0, 0, 8},
};

BNO055 bno(&hi2c1);
Timer timer;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
        led0 = !led0;
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
    can.init();
}

void main_app() {
    setup();
    while (1) {
        can.send(canRecvData);
        led1 = !led1;
        led2 = !led2;
        printfDMA("Hello\n");
        HAL_Delay(100);
    }
}