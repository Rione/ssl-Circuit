#ifndef LSM6DSO32_H_
#define LSM6DSO32_H_

#include "main.h"
#include "spi.h"
#include <stdbool.h>
#include <stdint.h>

// STMicroelectronics LSM6DSO32XTR (6軸IMU: 加速度計+ジャイロ) SPIドライバ
// レジスタマップはLSM6DSOファミリ共通 (LSM6DSO/LSM6DSOX/LSM6DSO32/LSM6DSO32X)

#define LSM6DSO32_REG_WHO_AM_I 0x0F
#define LSM6DSO32_WHO_AM_I_VALUE 0x6C

#define LSM6DSO32_REG_CTRL1_XL 0x10
#define LSM6DSO32_REG_CTRL2_G 0x11
#define LSM6DSO32_REG_CTRL3_C 0x12
#define LSM6DSO32_REG_STATUS_REG 0x1E
#define LSM6DSO32_REG_OUT_TEMP_L 0x20

#define LSM6DSO32_CTRL3_C_SW_RESET 0x01
#define LSM6DSO32_CTRL3_C_IF_INC 0x04
#define LSM6DSO32_CTRL3_C_BDU 0x40

// CTRL1_XL / CTRL2_G の ODR (出力データレート) 設定値
typedef enum {
  LSM6DSO32_ODR_OFF = 0x00 << 4,
  LSM6DSO32_ODR_12_5HZ = 0x01 << 4,
  LSM6DSO32_ODR_26HZ = 0x02 << 4,
  LSM6DSO32_ODR_52HZ = 0x03 << 4,
  LSM6DSO32_ODR_104HZ = 0x04 << 4,
  LSM6DSO32_ODR_208HZ = 0x05 << 4,
  LSM6DSO32_ODR_416HZ = 0x06 << 4,
  LSM6DSO32_ODR_833HZ = 0x07 << 4,
} Lsm6dso32Odr;

// 加速度計フルスケール (LSM6DSO32は±2gの代わりに±32gを持つ点がLSM6DSOと異なる)
typedef enum {
  LSM6DSO32_ACCEL_FS_4G = 0x00 << 2,
  LSM6DSO32_ACCEL_FS_32G = 0x01 << 2,
  LSM6DSO32_ACCEL_FS_8G = 0x02 << 2,
  LSM6DSO32_ACCEL_FS_16G = 0x03 << 2,
} Lsm6dso32AccelFs;

// ジャイロフルスケール
typedef enum {
  LSM6DSO32_GYRO_FS_250DPS = 0x00 << 2,
  LSM6DSO32_GYRO_FS_500DPS = 0x01 << 2,
  LSM6DSO32_GYRO_FS_1000DPS = 0x02 << 2,
  LSM6DSO32_GYRO_FS_2000DPS = 0x03 << 2,
} Lsm6dso32GyroFs;

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *csPort;
  uint16_t csPin;

  float accelSensitivity; // g/LSB
  float gyroSensitivity;  // dps/LSB

  float accelX, accelY, accelZ; // g
  float gyroX, gyroY, gyroZ;    // dps
  float temperature;            // degC
} Lsm6dso32;

static inline void Lsm6dso32_WriteReg(Lsm6dso32 *self, uint8_t reg,
                                      uint8_t data) {
  uint8_t tx[2] = {(uint8_t)(reg & 0x7F), data};
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(self->hspi, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_SET);
}

static inline uint8_t Lsm6dso32_ReadReg(Lsm6dso32 *self, uint8_t reg) {
  uint8_t tx = (uint8_t)(reg | 0x80);
  uint8_t rx = 0;
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(self->hspi, &tx, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(self->hspi, &rx, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_SET);
  return rx;
}

static inline void Lsm6dso32_ReadRegs(Lsm6dso32 *self, uint8_t reg,
                                      uint8_t *buf, uint16_t len) {
  uint8_t tx = (uint8_t)(reg | 0x80);
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(self->hspi, &tx, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(self->hspi, buf, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(self->csPort, self->csPin, GPIO_PIN_SET);
}

// 初期化 (CSピンはSPIペリフェラルのGPIO初期化後にソフトウェアCSとして呼び出し側で
// GPIO_MODE_OUTPUT_PPに設定しておくこと)
static inline bool Lsm6dso32_Init(Lsm6dso32 *self, SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *csPort, uint16_t csPin,
                                  Lsm6dso32Odr accelOdr,
                                  Lsm6dso32AccelFs accelFs,
                                  Lsm6dso32Odr gyroOdr,
                                  Lsm6dso32GyroFs gyroFs) {
  self->hspi = hspi;
  self->csPort = csPort;
  self->csPin = csPin;
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

  if (Lsm6dso32_ReadReg(self, LSM6DSO32_REG_WHO_AM_I) !=
      LSM6DSO32_WHO_AM_I_VALUE) {
    return false;
  }

  // ソフトウェアリセット
  Lsm6dso32_WriteReg(self, LSM6DSO32_REG_CTRL3_C, LSM6DSO32_CTRL3_C_SW_RESET);
  while (Lsm6dso32_ReadReg(self, LSM6DSO32_REG_CTRL3_C) &
        LSM6DSO32_CTRL3_C_SW_RESET)
    ;

  // BDU=1 (読み出し中のMSB/LSB不整合防止), IF_INC=1 (連続読み出し用アドレス自動増加)
  Lsm6dso32_WriteReg(self, LSM6DSO32_REG_CTRL3_C,
                    LSM6DSO32_CTRL3_C_BDU | LSM6DSO32_CTRL3_C_IF_INC);

  Lsm6dso32_WriteReg(self, LSM6DSO32_REG_CTRL1_XL, accelOdr | accelFs);
  Lsm6dso32_WriteReg(self, LSM6DSO32_REG_CTRL2_G, gyroOdr | gyroFs);

  switch (accelFs) {
    case LSM6DSO32_ACCEL_FS_4G:
      self->accelSensitivity = 0.122e-3f;
      break;
    case LSM6DSO32_ACCEL_FS_8G:
      self->accelSensitivity = 0.244e-3f;
      break;
    case LSM6DSO32_ACCEL_FS_16G:
      self->accelSensitivity = 0.488e-3f;
      break;
    case LSM6DSO32_ACCEL_FS_32G:
      self->accelSensitivity = 0.976e-3f;
      break;
  }
  switch (gyroFs) {
    case LSM6DSO32_GYRO_FS_250DPS:
      self->gyroSensitivity = 8.75e-3f;
      break;
    case LSM6DSO32_GYRO_FS_500DPS:
      self->gyroSensitivity = 17.5e-3f;
      break;
    case LSM6DSO32_GYRO_FS_1000DPS:
      self->gyroSensitivity = 35.0e-3f;
      break;
    case LSM6DSO32_GYRO_FS_2000DPS:
      self->gyroSensitivity = 70.0e-3f;
      break;
  }

  return true;
}

// 新規データ有無 (加速度・ジャイロのいずれかが更新済みならtrue)
static inline bool Lsm6dso32_DataReady(Lsm6dso32 *self) {
  return (Lsm6dso32_ReadReg(self, LSM6DSO32_REG_STATUS_REG) & 0x03) != 0;
}

// 温度・ジャイロ・加速度をOUT_TEMP_Lからの連続読み出しで取得し、メンバ変数を更新する
static inline void Lsm6dso32_Update(Lsm6dso32 *self) {
  uint8_t buf[14];
  Lsm6dso32_ReadRegs(self, LSM6DSO32_REG_OUT_TEMP_L, buf, sizeof(buf));

  int16_t rawTemp = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawGx = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawGy = (int16_t)((buf[5] << 8) | buf[4]);
  int16_t rawGz = (int16_t)((buf[7] << 8) | buf[6]);
  int16_t rawAx = (int16_t)((buf[9] << 8) | buf[8]);
  int16_t rawAy = (int16_t)((buf[11] << 8) | buf[10]);
  int16_t rawAz = (int16_t)((buf[13] << 8) | buf[12]);

  self->temperature = (float)rawTemp / 256.0f + 25.0f;
  self->gyroX = (float)rawGx * self->gyroSensitivity;
  self->gyroY = (float)rawGy * self->gyroSensitivity;
  self->gyroZ = (float)rawGz * self->gyroSensitivity;
  self->accelX = (float)rawAx * self->accelSensitivity;
  self->accelY = (float)rawAy * self->accelSensitivity;
  self->accelZ = (float)rawAz * self->accelSensitivity;
}

#endif
