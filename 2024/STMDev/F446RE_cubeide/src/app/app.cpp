#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU9250.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU9250 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

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

    mpu.setGyroFullScaleRange(GFSR_500DPS);
    mpu.setAccFullScaleRange(AFSR_4G);
    mpu.setDeltaTime(0.004);
    mpu.setTau(0.98);

    if (mpu.begin() != true) {
        printf("MPU9250 initialization failed\n");
        while (1) {
            robot.led0 = !robot.led0;
            HAL_Delay(100);
        }
    }

    printf("CALIBRATING\n");
    mpu.calibrateGyro(1500);
}

void main_app() {
    setup();
    while (1) {
        RawData imu = mpu.readRawData();
        printf("x: %d, y: %d, z: %d\n", imu.ax, imu.ay, imu.az);
        // mainMode.loop();
    }
}