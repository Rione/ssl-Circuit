#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

Timer time;
int cnt = 0;

void mpuget() {
    MPU6500::acc_t acc;
    MPU6500::gyro_t gyro;

    mpu.readAccGyro(&acc, &gyro);
    cnt++;

    if (time.read_ms() > 1000) {
        printf("cnt : %d\n", cnt);
        cnt = 0;
        time.reset();
    }
}

void TimInterrupt1khz() {
    robot.heartBeat();
}

void TimInterrupt4khz() {
    mpuget();
}

void canRxInterrupt(CAN_HandleTypeDef *hcan) {
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
    while (mpu.init() == 0) {
        robot.led0 = !robot.led0;
        printf("MPU6500 init failed\n");
    }
    robot.hardwareInit();
}

void main_app() {
    setup();
    while (1) {
        robot.led1 = !robot.led1;
        HAL_Delay(1000);
        // mainMode.loop();
    }
}