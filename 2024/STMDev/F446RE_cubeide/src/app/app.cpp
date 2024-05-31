#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"
#include "MadgwickAHRS.h"
#include "FLASH_EEPROM.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
Flash_EEPROM flash;

MPU6500::xyz_t vel;
MPU6500::acc_t acc;
MPU6500::gyro_t gyro;
MPU6500::xyz_t att;

Madgwick filter;
int32_t d = 0;
float frontDeg = 0;

void mpuget() {

    if (mpu.isCalibrated() == true) {
        mpu.getAccGyro(&acc, &gyro, false);
        filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);

        att.x = filter.getRoll();
        att.y = filter.getPitch();
        att.z = MyMath::gapDegrees180(filter.getYaw() > 180 ? filter.getYaw() - 360 : filter.getYaw(), frontDeg);

        // printf("ax,%.6f,ay,%.6f,az,%.6f,  ", acc.x, acc.y, acc.z);
        // printf("gx,%.6f,gy, %.6f,gz,%.6f", gyro.x, gyro.y, gyro.z);
        printf("yaw,%.6f", att.z);
        printf("\n");
        d++;
        robot.motorDriver.setVelocityFF(0, 1000 * MyMath::cosDeg(d * 0.36), att.z * -200);
    }

    // vel.x += acc.x;
    // vel.y += acc.y;
    // vel.z += acc.z;
}

void TimInterrupt1khz() {
    robot.heartBeat();
    robot.swImu.update();
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

    typedef struct {
        MPU6500::acc_t acc;
        MPU6500::gyro_t gyro;
    } imuOffsets_t;

    imuOffsets_t imuOffsets;

    if (robot.swImu.read() == false && mpu.init() == true) {
        // set flash
        printf("IMU calibrating\n");
        mpu.calibrateAccGyro();
        mpu.getOffset(&imuOffsets.acc, &imuOffsets.gyro);
        flash.writeFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        HAL_Delay(1000);
        flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        printf("ACC offset saved %.6f, %.6f, %.6f\n", imuOffsets.acc.x, imuOffsets.acc.y, imuOffsets.acc.z);
        printf("GYR offset saved %.6f, %.6f, %.6f\n", imuOffsets.gyro.x, imuOffsets.gyro.y, imuOffsets.gyro.z);
    } else {
        // load flash
        flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        mpu.setOffset(&imuOffsets.acc, &imuOffsets.gyro);
        printf("ACC offset loaded %.6f, %.6f, %.6f\n", imuOffsets.acc.x, imuOffsets.acc.y, imuOffsets.acc.z);
        printf("GYR offset loaded %.6f, %.6f, %.6f\n", imuOffsets.gyro.x, imuOffsets.gyro.y, imuOffsets.gyro.z);
        HAL_Delay(1000);
    }

    filter.begin(4000);
    robot.hardwareInit();
}

void main_app() {
    frontDeg = att.z;
    while (1) {
        // printf("x: %.2f, y: %.2f, z: %.2f ", vel.x, vel.y, vel.z);

        // wait_ms(10);

        // robot->info.velAngler.vel = 0;
        // robot->motorDriver.setVelocityFF(robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
        wait_ms(10);
        // mainMode.loop();
    }
}