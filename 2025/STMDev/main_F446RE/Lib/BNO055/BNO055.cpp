#include "BNO055.hpp"

// HALのI2C完了コールバックはグローバル関数なので、対応するインスタンスをここに覚えておく
static BNO055 *asyncInstance = nullptr;

BNO055::BNO055(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *sclPort, uint16_t sclPin, GPIO_TypeDef *sdaPort, uint16_t sdaPin)
    : _hi2c(hi2c), _sclPort(sclPort), _sclPin(sclPin), _sdaPort(sdaPort), _sdaPin(sdaPin) {
      asyncInstance = this;
}

void BNO055::recoverBus() {
      HAL_I2C_DeInit(_hi2c);  // ピンをI2Cから切り離す

      GPIO_InitTypeDef gpio = {};
      gpio.Mode = GPIO_MODE_OUTPUT_OD;
      gpio.Pull = GPIO_NOPULL;
      gpio.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_WritePin(_sclPort, _sclPin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(_sdaPort, _sdaPin, GPIO_PIN_SET);
      gpio.Pin = _sclPin;
      HAL_GPIO_Init(_sclPort, &gpio);
      gpio.Pin = _sdaPin;
      HAL_GPIO_Init(_sdaPort, &gpio);
      wait_us(5);

      // BNO055が送信途中のバイトを出し切るまでSCLを送る(SDAが解放されたら終わり)
      for (uint8_t i = 0; i < 9 && HAL_GPIO_ReadPin(_sdaPort, _sdaPin) == GPIO_PIN_RESET; i++) {
            HAL_GPIO_WritePin(_sclPort, _sclPin, GPIO_PIN_RESET);
            wait_us(5);
            HAL_GPIO_WritePin(_sclPort, _sclPin, GPIO_PIN_SET);
            wait_us(5);
      }
      // STOP条件 (SCL HighのままSDAをLow→High)
      HAL_GPIO_WritePin(_sdaPort, _sdaPin, GPIO_PIN_RESET);
      wait_us(5);
      HAL_GPIO_WritePin(_sdaPort, _sdaPin, GPIO_PIN_SET);
      wait_us(5);

      HAL_I2C_Init(_hi2c);  // MspInitでピンがI2Cに戻り、ペリフェラルもソフトウェアリセットされる
      asyncBusy = false;
}

bool BNO055::startReadAsync() {
      if (asyncBusy) {
            if (HAL_GetTick() - asyncStartTick < ASYNC_TIMEOUT_MS) {
                  return false;
            }
            // 完了もエラーも来ないまま止まった場合はバスごと復旧する
            busRecoverCount++;
            recoverBus();
      }
      if (__HAL_I2C_GET_FLAG(_hi2c, I2C_FLAG_BUSY)) {
            // バスがロックしている。このままHAL_I2C_Mem_Read_ITを呼ぶとBUSY解除を最大25ms待ってしまう
            busRecoverCount++;
            recoverBus();
            return false;
      }
      asyncBusy = true;
      asyncStartTick = HAL_GetTick();
      if (HAL_I2C_Mem_Read_IT(_hi2c, BNOAddress, BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, asyncBuf, ASYNC_LEN) != HAL_OK) {
            asyncBusy = false;
            return false;
      }
      return true;
}

bool BNO055::fetchAsync(acc_t *acc, gyro_t *gyro, euler_t *euler) {
      if (!asyncReady) {
            return false;
      }
      asyncReady = false;
      const uint8_t *d = asyncBuf;
      lastRawAllZero = true;
      static const uint8_t rawIdx[] = {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17};  // 加速度 d[0..5]・ジャイロ d[12..17]
      for (uint8_t i : rawIdx) {
            if (d[i] != 0) {
                  lastRawAllZero = false;
                  break;
            }
      }
      acc->x = int16_t(d[1] << 8 | d[0]) / 100.0f - offsetAcc.x;
      acc->y = int16_t(d[3] << 8 | d[2]) / 100.0f - offsetAcc.y;
      acc->z = int16_t(d[5] << 8 | d[4]) / 100.0f - offsetAcc.z;
      // d[6..11]はMAG(未使用)
      gyro->x = int16_t(d[13] << 8 | d[12]) / 16.0f - offsetGyro.x;
      gyro->y = int16_t(d[15] << 8 | d[14]) / 16.0f - offsetGyro.y;
      gyro->z = int16_t(d[17] << 8 | d[16]) / 16.0f - offsetGyro.z;
      // 1deg→16  1rad→900 (getEulerと同じ)
      euler->yaw = int16_t(d[19] << 8 | d[18]) / 900.0f;
      euler->roll = int16_t(d[21] << 8 | d[20]) / 900.0f;
      euler->pitch = int16_t(d[23] << 8 | d[22]) / 900.0f;
      return true;
}

void BNO055::onAsyncComplete() {
      asyncBusy = false;
      asyncReady = true;
}

void BNO055::onAsyncError() {
      i2cErrorCount++;
      asyncBusy = false;
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
      if (asyncInstance != nullptr && asyncInstance->getHi2c() == hi2c) {
            asyncInstance->onAsyncComplete();
      }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
      if (asyncInstance != nullptr && asyncInstance->getHi2c() == hi2c) {
            asyncInstance->onAsyncError();
      }
}

bool BNO055::check() {
      uint8_t data = 0;  // 応答が無いと読み出し先は書き換わらないので、偶然0xA0と判定しないよう初期化しておく
      if (HAL_I2C_Mem_Read(_hi2c, BNOAddress, BNO055_CHIP_ID_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK || data != 0xA0) {
            printf("BNO055 not found\n");
            return false;
      } else {
            printf("BNO055 found\n");
            return true;
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
      if (mode > 0x0C) {
            printf("Invalid mode\n");
            return;
      }
      HAL_I2C_Mem_Write(_hi2c, BNOAddress, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
}

bool BNO055::readOperationMode(uint8_t *mode) {
      return HAL_I2C_Mem_Read(_hi2c, BNOAddress, BNO055_OPR_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, mode, 1, 100) == HAL_OK;
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
      // printf("Euler: %.2f %.2f %.2f\n", euler.yaw, euler.roll, euler.pitch);
      return euler;
}

acc_t BNO055::getAcc() {
      acc_t acc;
      uint8_t data[6];
      HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
      acc.x = int16_t(data[1] << 8 | data[0]) / 100.0f - offsetAcc.x;
      acc.y = int16_t(data[3] << 8 | data[2]) / 100.0f - offsetAcc.y;
      acc.z = int16_t(data[5] << 8 | data[4]) / 100.0f - offsetAcc.z;
      // printfDMA("Acc: %.2f %.2f %.2f\n", acc.x, acc.y, acc.z);
      return acc;
}

gyro_t BNO055::getGyro() {
      gyro_t gyro;
      uint8_t data[6];
      HAL_I2C_Mem_Read(&hi2c1, BNOAddress, BNO055_GYRO_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
      gyro.x = int16_t(data[1] << 8 | data[0]) / 16.0f - offsetGyro.x;
      gyro.y = int16_t(data[3] << 8 | data[2]) / 16.0f - offsetGyro.y;
      gyro.z = int16_t(data[5] << 8 | data[4]) / 16.0f - offsetGyro.z;
      // printf("Gyro: %.2f %.2f %.2f\n", gyro.x, gyro.y, gyro.z);
      return gyro;
}

vel_t BNO055::getVel() {
      // 前回の加速度を保存しておく
      acc_t acc = getAcc();
      // dtを計算
      dt = dt_timer.read_us() / 1000000.0f;
      dt_timer.reset();

      // 加速度から速度を計算
      vel.x += (acc.x + prevAcc.x) * dt / 2.0f;
      vel.y += (acc.y + prevAcc.y) * dt / 2.0f;
      vel.z += (acc.z + prevAcc.z) * dt / 2.0f;

      // 前回の加速度を更新
      prevAcc = acc;

      return vel;
}

void BNO055::getOffset(acc_t *offset_acc, gyro_t *offset_gyro) {
      acc_t acc;
      acc_t acc_sum = {0, 0, 0};
      gyro_t gyro;
      gyro_t gyro_sum = {0, 0, 0};
      for (uint16_t i = 0; i < cnt_calib; i++) {
            acc = getAcc();
            gyro = getGyro();
            acc_sum.x += acc.x;
            acc_sum.y += acc.y;
            acc_sum.z += acc.z;
            gyro_sum.x += gyro.x;
            gyro_sum.y += gyro.y;
            gyro_sum.z += gyro.z;
            wait_us(100);
      }

      offset_acc->x = acc_sum.x / cnt_calib;
      offset_acc->y = acc_sum.y / cnt_calib;
      offset_acc->z = acc_sum.z / cnt_calib;
      offset_gyro->x = gyro_sum.x / cnt_calib;
      offset_gyro->y = gyro_sum.y / cnt_calib;
      offset_gyro->z = gyro_sum.z / cnt_calib;
}

void BNO055::setOffset(acc_t *acc, gyro_t *gyro) {
      offsetAcc = *acc;
      offsetGyro = *gyro;
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

// setAttitudeで設定した正面の角度を基準に相対角度を出す
float BNO055::getAttitude() {
      // frontとの差
      euler_t euler = getEuler();
      float attitude = euler.yaw - frontRadians;
      while (attitude < 0)
            attitude += TWO_PI;
      while (attitude > PI)
            attitude -= TWO_PI;
      attitude = -attitude;
      return attitude;
}

// この関数を呼び出した時点で向いていた角度を正面の角度として設定する
void BNO055::setAttitudeZero() {
      euler_t euler = getEuler();
      frontRadians = euler.yaw;
}

void BNO055::setAngle(float rad) {
      frontRadians = getAttitude() + rad;
}