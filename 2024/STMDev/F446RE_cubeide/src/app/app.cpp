#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

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
    acc_at acc;
    gyro_at gyro;
    
    float mpu_time[2]={0};
    float mpu_time_diff=0;    

    setup();
    while (1) {
        if (mpu.init() == 0) {
            robot.led0 = !robot.led0;
            printf("MPU6500 init failed\n");
        }else{
            
            mpu_time[0] = HAL_GetTick();

            mpu.readAccGyro(&acc, &gyro);

            mpu_time[1] = HAL_GetTick();
            
            mpu_time_diff = mpu_time[1] - mpu_time[0];

            printf("%f\n", mpu_time_diff);



            // acc.x /= 2048;
            // acc.y /= 2048;
            // acc.z /= 2048;

            // printf("acc: %.2f, %.2f, %.2f\n", acc.x, acc.y, acc.z);

            // gyro.x /= 16.4;
            // gyro.y /= 16.4;
            // gyro.z /= 16.4;

            // printf("gyro: %.2f, %.2f, %.2f\n", gyro.x, gyro.y, gyro.z);
        }

        HAL_Delay(1000);

        // mainMode.loop();
    }
}