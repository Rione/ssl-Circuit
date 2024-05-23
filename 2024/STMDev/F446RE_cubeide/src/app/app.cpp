#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU9250.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU9250 imu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim10) {
        robot.heartBeat();
    } else {
        // pass
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (robot.can.getHcan() == hcan) {
        robot.can.recv(canRecvData);
        robot.led0 = !robot.led0;
        switch (canRecvData.stdId) {
        case 0x123: // フォトセンサの値
            robot.info.photoSensorValue = canRecvData.data[0];
            break;
        default:
            break;
        }
    }
}

void setup() {
    robot.hardwareInit();
}

void main_app() {
    setup();
    while (1) {
        // mainMode.loop();
    }
}