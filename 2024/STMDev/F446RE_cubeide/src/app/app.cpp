#include "app.h"
#include "main.h"
#include "DMAStream.h"
#include "CAN.hpp"
#include "Timer.hpp"
#include "i2c.h"
#include "BNO055.hpp"
#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "BufferedSerial.hpp"

CANBus can(&hcan1, 0x100);
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);
DigitalOut led1(LED1_GPIO_Port, LED1_Pin);
DigitalOut led2(LED2_GPIO_Port, LED2_Pin);
PwmSingleOut ledH(&htim1, TIM_CHANNEL_1);
BufferedSerial serial1(&huart1, 128);
BufferedSerial serial4(&huart4, 128);
BufferedSerial serial5(&huart5, 128);

CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {100, 200, 0, 0, 0, 0, 0, 8},
};

BNO055 bno(&hi2c1);
Timer timer;
int i = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
        led0 = !led0;
    }
}

inline __attribute__((always_inline)) void heartBeat() {
    i++;
    ledH.write(MyMath::sinDeg(int(i / 5)) / 2 + 0.5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim10) {
        heartBeat(); // 1ms
    } else {
        // pass
    }
}

void setup() {
    bno.check();
    bno.setUnit(1, 1, 1, 0);
    bno.setPowerMode();
    bno.setOperaitonMode(OPERATION_MODE_AMG);
    bno.accConfig();
    bno.init();
    can.init();
    serial1.init();
    serial4.init();
    serial5.init();
    ledH.init();
    printf("Hello World\n");
}

void main_app() {
    setup();
    while (1) {
        if (serial4.available()) {
            uint8_t data = serial4.read();
            printfDMA("recive:%c\n", data); // なぜかDMAStreamを使わないとprintfが使えない
            led0 = !led0;
        }
        HAL_Delay(100);
        // printf("Hello World\n");
    }
}