#ifndef MPU6500_HPP
#define MPU6500_HPP

#include <stdint.h>
#include "Timer.hpp"
#include "MyMath.hpp"
#include "spi.h"

#define REG_XG_OFFS_TC 0x00
#define REG_YG_OFFS_TC 0x01
#define REG_ZG_OFFS_TC 0x02
#define REG_X_FINE_GAIN 0x03
#define REG_Y_FINE_GAIN 0x04
#define REG_Z_FINE_GAIN 0x05
#define REG_XA_OFFS_H 0x06
#define REG_XA_OFFS_L 0x07
#define REG_YA_OFFS_H 0x08
#define REG_YA_OFFS_L 0x09
#define REG_ZA_OFFS_H 0x0A
#define REG_ZA_OFFS_L 0x0B
#define REG_PRODUCT_ID 0x0C
#define REG_SELF_TEST_X 0x0D
#define REG_SELF_TEST_Y 0x0E
#define REG_SELF_TEST_Z 0x0F
#define REG_SELF_TEST_A 0x10
#define REG_XG_OFFS_USRH 0x13
#define REG_XG_OFFS_USRL 0x14
#define REG_YG_OFFS_USRH 0x15
#define REG_YG_OFFS_USRL 0x16
#define REG_ZG_OFFS_USRH 0x17
#define REG_ZG_OFFS_USRL 0x18
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_YOUT_L 0x46
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_BANK_SEL 0x6D
#define REG_MEM_START_ADDR 0x6E
#define REG_MEM_R_W 0x6F
#define REG_DMP_CFG_1 0x70
#define REG_DMP_CFG_2 0x71
#define REG_FIFO_COUNTH 0x72
#define REG_FIFO_COUNTL 0x73
#define REG_FIFO_R_W 0x74
#define REG_WHOAMI 0x75

#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS 0x00
#define BITS_FS_500DPS 0x08
#define BITS_FS_1000DPS 0x10
#define BITS_FS_2000DPS 0x18
#define BITS_FS_2G 0x00
#define BITS_FS_4G 0x08
#define BITS_FS_8G 0x10
#define BITS_FS_16G 0x18
#define BITS_FS_MASK 0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2 0x00
#define BITS_DLPF_CFG_188HZ 0x01
#define BITS_DLPF_CFG_98HZ 0x02
#define BITS_DLPF_CFG_42HZ 0x03
#define BITS_DLPF_CFG_20HZ 0x04
#define BITS_DLPF_CFG_10HZ 0x05
#define BITS_DLPF_CFG_5HZ 0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF 0x07
#define BITS_DLPF_CFG_MASK 0x07
#define BIT_INT_ANYRD_2CLEAR 0x10
#define BIT_RAW_RDY_EN 0x01
#define BIT_I2C_IF_DIS 0x10

#define READ_FLAG 0x80

class MPU6500 {
  public:
    typedef struct {
        float x;
        float y;
        float z;
    } acc_t;

    typedef struct {
        float x;
        float y;
        float z;
    } gyro_t;

    typedef struct {
        float x;
        float y;
        float z;
    } xyz_t;

    MPU6500(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    bool init(); // return 1 if success, -1 if fail
    void getAccGyro(acc_t *acc, gyro_t *gyro, bool divide);
    void readAccGyro(acc_t *acc, gyro_t *gyro);
    void calibrateAccGyro(acc_t *acc, gyro_t *gyro);
    void getOffset(acc_t *acc, gyro_t *gyro);
    void setOffset(acc_t *acc, gyro_t *gyro);

    bool calib;

  private:
    SPI_HandleTypeDef *_spi;
    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;

    const uint16_t cnt_calib = 34464;
    float initial_acc[3] = {0};  // calibration acc x,y,z
    float initial_gyro[3] = {0}; // calibration gyro x,y,z

    void read_reg(uint8_t reg, uint8_t *data, size_t length) {
        uint8_t tx_data[length + 1];
        uint8_t rx_data[length + 1];

        tx_data[0] = reg | READ_FLAG;
        for (size_t i = 1; i <= length; i++) {
            tx_data[i] = 0x00; // dummy data for reading
        }

        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(_spi, tx_data, rx_data, length + 1, 1);
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);

        for (size_t i = 0; i < length; i++) {
            data[i] = rx_data[i + 1];
        }
    }

    void write_reg(uint8_t reg, uint8_t data) {
        uint8_t tx_data[2];
        uint8_t rx_data[2];

        tx_data[0] = reg & 0x7F;
        tx_data[1] = data;

        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(_spi, tx_data, rx_data, 2, 1);
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
    }
};

#endif