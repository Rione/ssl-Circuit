#include "app.h"
#include "main.h"
#include "DMAStream.h"
#include "CAN.hpp"
#include "Timer.hpp"
#include "i2c.h"

#define BNOAddress (0x28 << 1) // GY-BNO055=0x29,Normal=0x28
// Register definitions
/* Page id register definition */
#define BNO055_PAGE_ID_ADDR 0x07
/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0x06
/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D
/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0x13
/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19
/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B
#define BNO055_EULER_R_LSB_ADDR 0x1C
#define BNO055_EULER_R_MSB_ADDR 0x1D
#define BNO055_EULER_P_LSB_ADDR 0x1E
#define BNO055_EULER_P_MSB_ADDR 0x1F
/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27
/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D
/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0x33
/* Temperature data register */
#define BNO055_TEMP_ADDR 0x34
/* Status registers */
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_SELFTEST_RESULT_ADDR 0x36
#define BNO055_INTR_STAT_ADDR 0x37
#define BNO055_SYS_CLK_STAT_ADDR 0x38
#define BNO055_SYS_STAT_ADDR 0x39
#define BNO055_SYS_ERR_ADDR 0x3A
/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_DATA_SELECT_ADDR 0x3C
/* Mode registers */
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_TEMP_SOURCE_ADDR 0x40
/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42
/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR 0x55
#define ACCEL_OFFSET_X_MSB_ADDR 0x56
#define ACCEL_OFFSET_Y_LSB_ADDR 0x57
#define ACCEL_OFFSET_Y_MSB_ADDR 0x58
#define ACCEL_OFFSET_Z_LSB_ADDR 0x59
#define ACCEL_OFFSET_Z_MSB_ADDR 0x5A
/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR 0x5B
#define MAG_OFFSET_X_MSB_ADDR 0x5C
#define MAG_OFFSET_Y_LSB_ADDR 0x5D
#define MAG_OFFSET_Y_MSB_ADDR 0x5E
#define MAG_OFFSET_Z_LSB_ADDR 0x5F
#define MAG_OFFSET_Z_MSB_ADDR 0x60
/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR 0x61
#define GYRO_OFFSET_X_MSB_ADDR 0x62
#define GYRO_OFFSET_Y_LSB_ADDR 0x63
#define GYRO_OFFSET_Y_MSB_ADDR 0x64
#define GYRO_OFFSET_Z_LSB_ADDR 0x65
#define GYRO_OFFSET_Z_MSB_ADDR 0x66
/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR 0x67
#define ACCEL_RADIUS_MSB_ADDR 0x68
#define MAG_RADIUS_LSB_ADDR 0x69
#define MAG_RADIUS_MSB_ADDR 0x6A

/* Page 1 registers */
#define BNO055_UNIQUE_ID_ADDR 0x50

// Definitions for unit selection
#define MPERSPERS 0x00
#define MILLIG 0x01
#define DEG_PER_SEC 0x00
#define RAD_PER_SEC 0x02
#define DEGREES 0x00
#define RADIANS 0x04
#define CENTIGRADE 0x00
#define FAHRENHEIT 0x10
#define WINDOWS 0x00
#define ANDROID 0x80

// Definitions for power mode
#define POWER_MODE_NORMAL 0x00
#define POWER_MODE_LOWPOWER 0x01
#define POWER_MODE_SUSPEND 0x02

// Definitions for operating mode
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_ACCONLY 0x01
#define OPERATION_MODE_MAGONLY 0x02
#define OPERATION_MODE_GYRONLY 0x03
#define OPERATION_MODE_ACCMAG 0x04
#define OPERATION_MODE_ACCGYRO 0x05
#define OPERATION_MODE_MAGGYRO 0x06
#define OPERATION_MODE_AMG 0x07
#define OPERATION_MODE_IMUPLUS 0x08
#define OPERATION_MODE_COMPASS 0x09
#define OPERATION_MODE_M4G 0x0A
#define OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define OPERATION_MODE_NDOF 0x0C

Timer timer;

CANBus can(&hcan1, 0x100);
CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {1, 2, 0, 0, 0, 0, 0, 0},
};

// Timer interval;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
    }
}

void BNO055Check() {
    // BNO055が接続されているか確認
    // BNO055のチップIDを読み取り、0xA0であればBNO055が接続されていると判断
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    // Page 60 4.3.1 CHIP_ID
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_CHIP_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (data != 0xA0) {
        printf("BNO055 not found\n");
    } else {
        printf("BNO055 found\n");
    }
}

void BNO055Init() {
    uint8_t tx[1] = {BNO055_CHIP_ID_ADDR};
    uint8_t rx[16] = {0};
    HAL_I2C_Master_Transmit(&hi2c1, BNOAddress, tx, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, BNOAddress + 1, rx, 7, 100);

    printf("Chip ID: %d\n", rx[0]);
    printf("acc: %d\n", rx[1]);
    printf("mag: %d\n", rx[2]);
    printf("gyro: %d\n", rx[3]);
    printf("sw0: %d\n", rx[4]);
    printf("sw1: %d\n", rx[5]);
    printf("bootload: %d\n", rx[6]);

    tx[0] = 1;
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);

    tx[0] = BNO055_UNIQUE_ID_ADDR;
    HAL_I2C_Master_Transmit(&hi2c1, BNOAddress, tx, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, BNOAddress + 1, rx, 16, 100);

    printf("UID: %d\n", rx[0]);

    tx[0] = 0;
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_PAGE_ID_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055SetPowerModeAsNormal() {
    uint8_t tx[1] = {POWER_MODE_NORMAL};
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_PWR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055SetOperationModeAsNDOF() {
    uint8_t tx[1] = {OPERATION_MODE_NDOF};
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055SetOperationModeAsAMG() {
    uint8_t tx[1] = {OPERATION_MODE_AMG};
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055SetCalibration() {
    uint8_t tx[1] = {0b00000000};
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055SetUnit() {
    // bit 1 : ACC  (0:m/s^2, 1:mg)
    // bit 2 : gyro (0:dps, 1:rps)
    // bit 3 : euler(0:degree, 1:radian)
    // bit 5 : temp (0:celsius, 1:fahrenheit)

    // euler angle unit : radian
    // accel unit : m/s^2
    // rate unit : rad/s

    uint8_t tx[1] = {0b00001100};
    HAL_I2C_Mem_Write(&hi2c1, BNOAddress, BNO055_UNIT_SEL_ADDR, I2C_MEMADD_SIZE_8BIT, tx, 1, 100);
}

void BNO055ReadEuler() {
    timer.reset();
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_EULER_H_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    // 1deg→16  1rad→900
    float angle_scale = 1.0f / 900.0f;
    float yaw = int16_t(data[1] << 8 | data[0]) * angle_scale;
    float roll = int16_t(data[3] << 8 | data[2]) * angle_scale;
    float pitch = int16_t(data[5] << 8 | data[4]) * angle_scale;
    printfDMA("Euler: %.2f %.2f %.2f time:%ld\n", yaw, roll, pitch, timer.read_us());
}

void BNO055ReadAcc() {
    timer.reset();
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    // 1m/s^2→100 1mg→1
    float accel_scale = 1.0f / 100.0f;
    float x = int16_t(data[1] << 8 | data[0]) * accel_scale;
    float y = int16_t(data[3] << 8 | data[2]) * accel_scale;
    float z = int16_t(data[5] << 8 | data[4]) * accel_scale;
    printfDMA("Accel: %.2f %.2f %.2f\n", x, y, z);
}

void BNO055ReadGyro() {
    timer.reset();
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_GYRO_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    // 1deg/s→16 1rad/s→900
    float rate_scale = 1.0f / 900.0f;
    float x = int16_t(data[1] << 8 | data[0]) * rate_scale;
    float y = int16_t(data[3] << 8 | data[2]) * rate_scale;
    float z = int16_t(data[5] << 8 | data[4]) * rate_scale;
    printfDMA("Gyro: %.2f %.2f %.2f time:%ld\n", x, y, z, timer.read_us());
}

void BNO055AccConfig() {
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

void BNO055GetCalibration() {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    // [SYS Calib Status] : [7,6 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [GYR Calib Status] : [5,4 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [ACC Calib Status] : [3,2 bit] : 0 or 3, Read 3 indicates fully calibrated
    // [MAG Calib Status] : [1,0 bit] : 0 or 3, Read 3 indicates fully calibrated
    uint8_t sys = (data >> 6) & 0x03;
    uint8_t gyr = (data >> 4) & 0x03;
    uint8_t acc = (data >> 2) & 0x03;
    uint8_t mag = (data >> 0) & 0x03;
    printf("Calibration sys:%d gyr:%d acc:%d mag:%d\n", sys, gyr, acc, mag);
}

void setup() {
    printf("Start %d %d\n", (BNOAddress + 1) << 1, ((BNOAddress) << 1) + 1);
}

void main_app() {

    setup();
    for (size_t i = 0; i < 10; i++) {
        BNO055Check();
    }
    BNO055SetUnit();              // 単位を設定
    BNO055SetPowerModeAsNormal(); // 通常モードに設定
    // BNO055SetOperationModeAsNDOF(); // FusionモードのNDOFに設定
    BNO055SetOperationModeAsAMG();
    BNO055AccConfig();
    BNO055Init();

    while (1) {

        // BNO055ReadEuler();
        BNO055ReadAcc();
        // BNO055ReadGyro();
        HAL_Delay(2);
    }
}