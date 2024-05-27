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


void mpuget(){
    acc_at acc;
    gyro_at gyro;
    
    
    int mpu_time_diff = 0;  

    mpu.readAccGyro(&acc, &gyro);

    acc.x /= 2048;
    acc.y /= 2048;
    acc.z /= 2048;
    // printfDMA("acc: %.2f, %.2f, %.2f\n", acc.x, acc.y, acc.z);

    gyro.x /= 16.4;
    gyro.y /= 16.4;
    gyro.z /= 16.4;
    // printfDMA("gyro: %.2f, %.2f, %.2f\n", gyro.x, gyro.y, gyro.z);

    cnt++;

    if(time.read_ms() > 1000){
        printf("cnt : %d\n", cnt);
        cnt = 0;
        time.reset();
    }
    
    // printf("time : %d\n", mpu_time_diff);  

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim10) {
        robot.heartBeat();
    } else if(htim == &htim3) {
        // 4KHz
        mpuget();
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

    while(mpu.init() == 0) {
            robot.led0 = !robot.led0;
            printf("MPU6500 init failed\n");
    }

    while (1) {
        robot.led1 = !robot.led1;
        HAL_Delay(1000);
        // mainMode.loop();
    }
}