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

MPU6500::acc_t acc;
MPU6500::gyro_t gyro;
MPU6500::xyz_t att;

Madgwick filter;
float frontDeg = 0;

void mpuget() {
    if (mpu.isCalibrated() == true) {
        mpu.getAccGyro(&acc, &gyro, false);
        filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
        att.z = (float)MyMath::gapDegrees180(filter.getYaw(), frontDeg);
    }
}

void TimInterrupt1khz() {
    robot.heartBeat();
    robot.swImu.update();
    robot.swDischarge.update();
}

void TimInterrupt4khz() {
    mpuget();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (robot.can.getHcan() == hcan) {
        robot.can.recv(canRecvData);
        switch (canRecvData.stdId) {
        // KickerBoard Commands
        case 0x10: // 16: charge Enable
            robot.led2 = true;
            break;
        case 0x11: // 17: charge Disable
            robot.minusCapChargeCertitude(100);
            robot.led2 = false;
            break;
        case 0x12: // 18: kick
            robot.minusCapChargeCertitude(canRecvData.data[0]);
            break;
        case 0x13: // 19: chip kick
            robot.minusCapChargeCertitude(canRecvData.data[0]);
            break;
        // case 0x14: // 20: dribbler run
        //     break;
        // case 0x15: // 21: dribbler stop
        //     break;
        case 0x123: // フォトセンサの値・充電完了信号の受信
            robot.setPhotoSensorValue((uint16_t)(canRecvData.data[0]) | (uint16_t)(canRecvData.data[1]) << 8);
            robot.setChageDoneSignal(canRecvData.data[2]); // 充電完了信号の処理
            robot.led0 = !robot.led0;
            break;
        default:
            break;
        }
    }
}

void setup() {
    // while (mpu.init() == 0) {
    //     robot.led0 = !robot.led0;
    //     printf("MPU6500 init failed\n");
    // }

    typedef struct {
        MPU6500::acc_t acc;
        MPU6500::gyro_t gyro;
    } imuOffsets_t;

    imuOffsets_t imuOffsets;

    if (robot.swImu.read() == false) {
        // set flash
        printf("IMU calibrating\n");
        HAL_Delay(1000);
        mpu.calibrateAccGyro();
        mpu.getOffset(&imuOffsets.acc, &imuOffsets.gyro);
        flash.writeFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        HAL_Delay(1000);
        flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        printf("ACC offset saved %.6f, %.6f, %.6f\n", imuOffsets.acc.x, imuOffsets.acc.y, imuOffsets.acc.z);
        printf("GYR offset saved %.6f, %.6f, %.6f\n", imuOffsets.gyro.x, imuOffsets.gyro.y, imuOffsets.gyro.z);
    } else {
        // load flash オフセット値をFlashから読み出す(初回起動時はimu resetスイッチを押して起動すること)
        flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&imuOffsets, sizeof(imuOffsets_t));
        mpu.setOffset(&imuOffsets.acc, &imuOffsets.gyro);
        printf("ACC offset loaded %.6f, %.6f, %.6f\n", imuOffsets.acc.x, imuOffsets.acc.y, imuOffsets.acc.z);
        printf("GYR offset loaded %.6f, %.6f, %.6f\n", imuOffsets.gyro.x, imuOffsets.gyro.y, imuOffsets.gyro.z);
        HAL_Delay(1000);
    }

    filter.begin(33000); // 4000のはずなんだけど、何故か8.25倍しないとmain_appで使い物にならなかった...
    robot.hardwareInit();

    robot.dribble(0);
}

void main_app() {
    frontDeg = att.z;
    while (1) {
        mainMode.loop();
        // printf("ball:%d Batt:%d\n", robot.info.photoSensorValue, robot.info.batteryVoltage);
        // HAL_Delay(10);
    }
}