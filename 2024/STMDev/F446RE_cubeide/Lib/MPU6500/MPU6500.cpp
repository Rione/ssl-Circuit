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
    HAL_Delay(10);

    // ACCEL 16G
    write_reg(REG_ACCEL_CONFIG, BITS_FS_16G);
    HAL_Delay(10);

    // sample rate divider : reset
    write_reg(REG_SMPLRT_DIV, 0x00);
    HAL_Delay(10);

    return 1; // success to recognize MPU6500
}

void MPU6500::readAccGyro(acc_t *acc, gyro_t *gyro) {
    uint8_t data[6];
    read_reg(REG_ACCEL_XOUT_H, data, 6);
    acc->x = (float)(int16_t)((data[0] << 8) | data[1]) / 2048;
    acc->y = (float)(int16_t)((data[2] << 8) | data[3]) / 2048;
    acc->z = (float)(int16_t)((data[4] << 8) | data[5]) / 2048;

    read_reg(REG_GYRO_XOUT_H, data, 6);
    gyro->x = (float)(int16_t)((data[0] << 8) | data[1]) / 16.4;
    gyro->y = (float)(int16_t)((data[2] << 8) | data[3]) / 16.4;
    gyro->z = (float)(int16_t)((data[4] << 8) | data[5]) / 16.4;
}