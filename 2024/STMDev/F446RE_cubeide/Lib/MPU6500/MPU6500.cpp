#include "MPU6500.hpp"

MPU6500::MPU6500(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
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

void MPU6500::calibrateAccGyro(acc_t *acc, gyro_t *gyro) {
    acc_t acc_sum = {0};
    gyro_t gyro_sum = {0};

    for (uint16_t i = 0; i < cnt_calib; i++) {
        readAccGyro(acc, gyro);
        acc_sum.x += acc->x;
        acc_sum.y += acc->y;
        acc_sum.z += acc->z;
        gyro_sum.x += gyro->x;
        gyro_sum.y += gyro->y;
        gyro_sum.z += gyro->z;
        // HAL_Delay();
        wait_us(500);
    }

    initial_acc[0] = acc_sum.x / cnt_calib;
    initial_acc[1] = acc_sum.y / cnt_calib;
    initial_acc[2] = acc_sum.z / cnt_calib;
    initial_gyro[0] = gyro_sum.x / cnt_calib;
    initial_gyro[1] = gyro_sum.y / cnt_calib;
    initial_gyro[2] = gyro_sum.z / cnt_calib;

    printf("initial_acc: %.2f, %.2f, %.2f\n", initial_acc[0], initial_acc[1], initial_acc[2]);
    printf("initial_gyro: %.2f, %.2f, %.2f\n", initial_gyro[0], initial_gyro[1], initial_gyro[2]);
    printf("calibration done\n");
    printf("\n");
    MPU6500::calib = true;
}

void MPU6500::getAccGyro(acc_t *acc, gyro_t *gyro, bool divide = false) {
    readAccGyro(acc, gyro);
    acc->x = (acc->x - initial_acc[0]) / (divide ? 16384 : 1);
    acc->y = (acc->y - initial_acc[1]) / (divide ? 16384 : 1);
    acc->z = (acc->z - initial_acc[2]) / (divide ? 16384 : 1);
    gyro->x = (gyro->x - initial_gyro[0]) / (divide ? 131 : 1);
    gyro->y = (gyro->y - initial_gyro[1]) / (divide ? 131 : 1);
    gyro->z = (gyro->z - initial_gyro[2]) / (divide ? 131 : 1);
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