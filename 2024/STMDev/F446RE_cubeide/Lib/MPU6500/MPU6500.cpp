#include "MPU6500.hpp"

MPU6500::MPU6500(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    _spi = spi;
    _cs_port = cs_port;
    _cs_pin = cs_pin;
}

bool MPU6500::init() {
    uint8_t check;
    read_reg(REG_WHOAMI, &check);
    if (check != 0x70) {
        return 0;
    }

    write_reg(REG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    write_reg(REG_CONFIG, 0x00);
    HAL_Delay(10);

    write_reg(REG_GYRO_CONFIG, BITS_FS_2000DPS);
    HAL_Delay(10);

    return 1; // success to recognize MPU6500
}

void readAccMag(xyz_t *acc, xyz_t *mag) {
    // uint8_t data[6];
    // read_reg(REG_ACCEL_XOUT_H, data);
    // acc->x = (int16_t)((data[0] << 8) | data[1]);
    // acc->y = (int16_t)((data[2] << 8) | data[3]);
    // acc->z = (int16_t)((data[4] << 8) | data[5]);

    // read_reg(REG_MAG_XOUT_L, data);
    // mag->x = (int16_t)((data[1] << 8) | data[0]);
    // mag->y = (int16_t)((data[3] << 8) | data[2]);
    // mag->z = (int16_t)((data[5] << 8) | data[4]);
}