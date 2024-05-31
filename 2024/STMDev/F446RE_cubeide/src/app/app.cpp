#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"
#include "MadgwickAHRS.h"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);

MPU6500::xyz_t vel;
MPU6500::acc_t acc;
MPU6500::gyro_t gyro;
MPU6500::xyz_t att;

Madgwick filter;

void mpuget() {

    if (mpu.calib == true) {
        mpu.getAccGyro(&acc, &gyro, false);
        filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);

        att.x = filter.getRoll();
        att.y = filter.getPitch();
        att.z = filter.getYaw() > 180 ? filter.getYaw() - 360 : filter.getYaw();

        // printf("ax,%.6f,ay,%.6f,az,%.6f,  ", acc.x, acc.y, acc.z);
        // printf("gx,%.6f,gy, %.6f,gz,%.6f", gyro.x, gyro.y, gyro.z);
        printf("yaw,%.6f", att.z);
        // float z = att.z;
        // if (z > 180) {
        //     z -= 360;
        // }
        // printf("yaw,%.6f\n", att.z);
        printf("\n");
        robot.motorDriver.setVelocityFF(robot.info.velX.vel, robot.info.velY.vel, att.z * -200);
    }

    // vel.x += acc.x;
    // vel.y += acc.y;
    // vel.z += acc.z;
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
    mpu.calib = false;
    filter.begin(4000);
    while (mpu.init() == 0) {
        robot.led0 = !robot.led0;
        printf("MPU6500 init failed\n");
    }

    if (mpu.init() == 1) {
        printf("MPU6500 init success\n");
        mpu.calibrateAccGyro(&acc, &gyro);
    }

    robot.hardwareInit();
}

void main_app() {
    setup();
    while (1) {
        // printf("x: %.2f, y: %.2f, z: %.2f ", vel.x, vel.y, vel.z);

        // wait_ms(10);

        // robot->info.velAngler.vel = 0;
        // robot->motorDriver.setVelocityFF(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
        wait_ms(10);
        // mainMode.loop();
    }
}