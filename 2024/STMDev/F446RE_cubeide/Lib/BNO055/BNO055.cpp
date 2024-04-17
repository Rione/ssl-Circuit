#include "BNO055.hpp"

BNO055::BNO055(I2C_HandleTypeDef *hi2c) : _hi2c(hi2c) {
}

void BNO055::check() {
    uint8_t data;
    HAL_I2C_Mem_Read(_hi2c, BNOAddress, BNO055_CHIP_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (data != 0xA0) {
        printf("BNO055 not found\n");
    } else {
        printf("BNO055 found\n");
    }
}

void BNO055::init() {
    uint8_t tx = BNO055_CHIP_ID_ADDR;
    uint8_t rx[16] = {0};
    HAL_I2C_Master_Transmit(_hi2c, BNOAddress, &tx, 1, 100);
    HAL_I2C_Master_Receive(_hi2c, BNOAddress + 1, rx, 7, 100);

    printf("Chip ID: %d\n", rx[0]);
    printf("acc: %d\n", rx[1]);
    printf("mag: %d\n", rx[2]);
    printf("gyro: %d\n", rx[3]);
    printf("sw0: %d\n", rx[4]);
    printf("sw1: %d\n", rx[5]);
    printf("bootload: %d\n", rx[6]);

    tx = 1;
    HAL_I2C_Mem_Write(_hi2c, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);

    tx = BNO055_UNIQUE_ID_ADDR;
    HAL_I2C_Master_Transmit(_hi2c, BNOAddress, &tx, 1, 100);
    HAL_I2C_Master_Receive(_hi2c, BNOAddress + 1, rx, 16, 100);

    printf("UID: %d\n", rx[0]);

    tx = 0;
    HAL_I2C_Mem_Write(_hi2c, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);
}

void BNO055::setPowerMode() {
    uint8_t tx = POWER_MODE_NORMAL;
    HAL_I2C_Mem_Write(_hi2c, BNOAddress, BNO055_PWR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);
}

void BNO055::setOperaitonMode(uint8_t mode) {
    // OPERATION_MODE_CONFIG        0x00
    // OPERATION_MODE_ACCONLY       0x01
    // OPERATION_MODE_MAGONLY       0x02
    // OPERATION_MODE_GYRONLY       0x03
    // OPERATION_MODE_ACCMAG        0x04
    // OPERATION_MODE_ACCGYRO       0x05
    // OPERATION_MODE_MAGGYRO       0x06
    // OPERATION_MODE_AMG           0x07
    // OPERATION_MODE_IMUPLUS       0x08
    // OPERATION_MODE_COMPASS       0x09
    // OPERATION_MODE_M4G           0x0A
    // OPERATION_MODE_NDOF_FMC_OFF  0x0B
    // OPERATION_MODE_NDOF          0x0C
    if (mode < 0x00 || mode > 0x0C) {
        printf("Invalid mode\n");
        return;
    }
    HAL_I2C_Mem_Write(_hi2c, BNOAddress, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
}

void BNO055::setUnit(bool acc, bool gyro, bool euler, bool temp) {
    // bit 1 : ACC  (0:m/s^2, 1:mg)
    // bit 2 : gyro (0:dps, 1:rps)
    // bit 3 : euler(0:degree, 1:radian)
    // bit 5 : temp (0:celsius, 1:fahrenheit)

    // euler angle unit : radian
    // accel unit : m/s^2
    // rate unit : rad/s

    // uint8_t tx = 0b00001100;
    uint8_t tx = 0b00000000;
    if (acc) {
        tx |= 0b00000001;
    }
    if (gyro) {
        tx |= 0b00000010;
    }
    if (euler) {
        tx |= 0b00000100;
    }
    if (temp) {
        tx |= 0b00010000;
    }

    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_UNIT_SEL_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);
}

void BNO055::accConfig() {
    // Page Change to 1
    uint8_t tx = 1;
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);

    // Range:16G Bandwidth:62.5Hz
    tx = 0b00001111;
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, 0x08, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);

    // Page Change to 1
    tx = 1;
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &tx, 1, 100);
}

euler_t BNO055::getEuler() {
    euler_t euler;
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_EULER_H_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    // 1deg→16  1rad→900
    float angle_scale = 1.0f / 900.0f;
    euler.yaw = int16_t(data[1] << 8 | data[0]) * angle_scale;
    euler.roll = int16_t(data[3] << 8 | data[2]) * angle_scale;
    euler.pitch = int16_t(data[5] << 8 | data[4]) * angle_scale;
    // printf("Euler: %.2f %.2f %.2f time:%ld\n", euler.yaw, euler.roll, euler.pitch, _timer.read_us());
    return euler;
}

acc_t BNO055::getAcc() {
    acc_t acc;
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    acc.x = int16_t(data[1] << 8 | data[0]) / 100.0f;
    acc.y = int16_t(data[3] << 8 | data[2]) / 100.0f;
    acc.z = int16_t(data[5] << 8 | data[4]) / 100.0f;
    // printf("Acc: %.2f %.2f %.2f time:%ld\n", acc.x, acc.y, acc.z, _timer.read_us());
    return acc;
}

gyro_t BNO055::getGyro() {
    gyro_t gyro;
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_GYRO_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    gyro.x = int16_t(data[1] << 8 | data[0]) / 16.0f;
    gyro.y = int16_t(data[3] << 8 | data[2]) / 16.0f;
    gyro.z = int16_t(data[5] << 8 | data[4]) / 16.0f;
    // printf("Gyro: %.2f %.2f %.2f time:%ld\n", gyro.x, gyro.y, gyro.z, _timer.read_us());
    return gyro;
}

void BNO055::getCalibration() {
    calib_t calib;
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    // [SYS Calib Status] : [7,6 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [GYR Calib Status] : [5,4 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [ACC Calib Status] : [3,2 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [MAG Calib Status] : [1,0 bit] : 0 or 3, Read 3 indicates fully calibrated
    calib.sys = (data >> 6) & 0x03;
    calib.gyr = (data >> 4) & 0x03;
    calib.acc = (data >> 2) & 0x03;
    calib.mag = (data >> 0) & 0x03;
    printf("Calibration sys:%d gyr:%d acc:%d mag:%d\n", calib.sys, calib.gyr, calib.acc, calib.mag);
}