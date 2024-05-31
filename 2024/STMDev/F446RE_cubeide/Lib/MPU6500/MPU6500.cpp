#include "MPU6500.hpp"

MPU6500::MPU6500(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin) : calib(false) {
    _spi = spi;
    _cs_port = cs_port;
    _cs_pin = cs_pin;
}

bool MPU6500::init() {
    uint8_t check;
    read_reg(REG_WHOAMI, &check, 1);
    if (check != 0x70) {
        return 0;
    }

    // CLOCK 0: internal 20MHz
    write_reg(REG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    // Bandwidth 250Hz
    write_reg(REG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);
    HAL_Delay(10);

    // GYRO 2000dps
    write_reg(REG_GYRO_CONFIG, BITS_FS_2000DPS);
    // write_reg(REG_GYRO_CONFIG, BITS_FS_250DPS); // 250dps
    HAL_Delay(10);

    // ACCEL 16G
    write_reg(REG_ACCEL_CONFIG, BITS_FS_16G);
    // write_reg(REG_ACCEL_CONFIG, BITS_FS_2G); // 2g
    HAL_Delay(10);

    // sample rate divider : reset
    write_reg(REG_SMPLRT_DIV, 0x00);
    HAL_Delay(10);

    return 1; // success to recognize MPU6500
}

void MPU6500::calibrateAccGyro() {
    acc_t acc;
    gyro_t gyro;

    acc_t acc_sum;
    gyro_t gyro_sum;

    for (uint16_t i = 0; i < cnt_calib; i++) {
        readAccGyro(&acc, &gyro);
        acc_sum.x += acc.x;
        acc_sum.y += acc.y;
        acc_sum.z += acc.z;
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;
        // HAL_Delay();
        wait_us(500);
    }

    offsetAcc.x = acc_sum.x / cnt_calib;
    offsetAcc.y = acc_sum.y / cnt_calib;
    offsetAcc.z = acc_sum.z / cnt_calib;
    offsetGyro.x = gyro_sum.x / cnt_calib;
    offsetGyro.y = gyro_sum.y / cnt_calib;
    offsetGyro.z = gyro_sum.z / cnt_calib;

    printf("initial_acc: %.2f, %.2f, %.2f\n", offsetAcc.x, offsetAcc.y, offsetAcc.z);
    printf("initial_gyro: %.2f, %.2f, %.2f\n", offsetGyro.x, offsetGyro.y, offsetGyro.z);
    printf("calibration done\n");
    printf("\n");

    calib = true;
}

void MPU6500::getAccGyro(acc_t *acc, gyro_t *gyro, bool divide = false) {
    readAccGyro(acc, gyro);
    acc->x = (acc->x - offsetAcc.x) / (divide ? 16384 : 1);
    acc->y = (acc->y - offsetAcc.y) / (divide ? 16384 : 1);
    acc->z = (acc->z - offsetAcc.z) / (divide ? 16384 : 1);
    gyro->x = (gyro->x - offsetGyro.x) / (divide ? 131 : 1);
    gyro->y = (gyro->y - offsetGyro.y) / (divide ? 131 : 1);
    gyro->z = (gyro->z - offsetGyro.z) / (divide ? 131 : 1);
}

void MPU6500::readAccGyro(acc_t *acc, gyro_t *gyro) {
    uint8_t data[6];
    read_reg(REG_ACCEL_XOUT_H, data, 6);
    acc->x = (float)(int16_t)((data[0] << 8) | data[1]);
    acc->y = (float)(int16_t)((data[2] << 8) | data[3]);
    acc->z = (float)(int16_t)((data[4] << 8) | data[5]);

    read_reg(REG_GYRO_XOUT_H, data, 6);
    gyro->x = (float)(int16_t)((data[0] << 8) | data[1]);
    gyro->y = (float)(int16_t)((data[2] << 8) | data[3]);
    gyro->z = (float)(int16_t)((data[4] << 8) | data[5]);
}

void MPU6500::getOffset(acc_t *acc, gyro_t *gyro) {
    acc->x = offsetAcc.x;
    acc->y = offsetAcc.y;
    acc->z = offsetAcc.z;
    gyro->x = offsetGyro.x;
    gyro->y = offsetGyro.y;
    gyro->z = offsetGyro.z;
}

void MPU6500::setOffset(acc_t *acc, gyro_t *gyro) {
    offsetAcc.x = acc->x;
    offsetAcc.y = acc->y;
    offsetAcc.z = acc->z;
    offsetGyro.x = gyro->x;
    offsetGyro.y = gyro->y;
    offsetGyro.z = gyro->z;
    calib = true;
}
