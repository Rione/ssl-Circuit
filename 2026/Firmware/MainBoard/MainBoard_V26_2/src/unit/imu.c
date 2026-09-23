#include "imu.h"

#include <stdio.h>

#include "main.h"
#include "mymath.h"
#include "parammeter.h"
#include "spi.h"

void Imu_Init(Imu* self) {
  self->accel_x = 0.0f;
  self->accel_y = 0.0f;
  self->yaw_rate = 0.0f;
  self->yaw_rad = 0.0f;
  self->gyro_bias_x = 0.0f;
  self->gyro_bias_y = 0.0f;
  self->gyro_bias_z = 0.0f;
  self->accel_bias_x = 0.0f;
  self->accel_bias_y = 0.0f;
  self->accel_robot_x = 0.0f;
  self->accel_robot_y = 0.0f;

  // ジャイロは±2000dps(ROBOT_MAX_ANG_VEL=10rad/s≈573dpsに対して十分な余裕を持たせる。
  // ±500dpsだと通常の最大角速度指令だけで飽和し、高速回転時にyawが追従しなくなる)
  self->is_ready =
      Lsm6dso32_Init(&self->sensor, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin,
                    LSM6DSO32_ODR_104HZ, LSM6DSO32_ACCEL_FS_4G,
                    LSM6DSO32_ODR_104HZ, LSM6DSO32_GYRO_FS_2000DPS);
  if (!self->is_ready) {
    printf("IMU Init Failed (WHO_AM_I mismatch)\n");
  }

  Madgwick_Init(&self->ahrs, IMU_MADGWICK_BETA);
  Timer_Init(&self->update_timer);
  Timer_Reset(&self->update_timer);
}

void Imu_Update(Imu* self) {
  if (!self->is_ready) return;
  if (!Lsm6dso32_DataReady(&self->sensor)) return;

  // 呼び出し周期がブロッキング処理(printf等)で乱れても正しく積分できるよう、
  // 前回呼び出しからの実経過時間を測定してMadgwickに渡す(固定dt厳禁)。
  float dt = Timer_Read(&self->update_timer);
  Timer_Reset(&self->update_timer);
  if (dt > IMU_MAX_DT_S) dt = IMU_MAX_DT_S;

  Lsm6dso32_Update(&self->sensor);

  float gyro_x = self->sensor.gyroX - self->gyro_bias_x;
  float gyro_y = self->sensor.gyroY - self->gyro_bias_y;
  float gyro_z = self->sensor.gyroZ - self->gyro_bias_z;

  self->accel_x = self->sensor.accelX;
  self->accel_y = self->sensor.accelY;
  float ax = self->sensor.accelX - self->accel_bias_x;
  float ay = self->sensor.accelY - self->accel_bias_y;
  self->accel_robot_x = IMU_TO_ROBOT_AX(ax, ay) * IMU_GRAVITY_MPS2;
  self->accel_robot_y = IMU_TO_ROBOT_AY(ax, ay) * IMU_GRAVITY_MPS2;
  self->yaw_rate = Radians(gyro_z);

  Madgwick_UpdateImu(&self->ahrs, Radians(gyro_x), Radians(gyro_y), self->yaw_rate,
                     self->sensor.accelX, self->sensor.accelY,
                     self->sensor.accelZ, dt);
  self->yaw_rad = Madgwick_GetYaw(&self->ahrs);
}

// ジャイロの静止バイアスをIMU_CALIB_SAMPLE_COUNT回サンプリングして平均する(ブロッキング)
void Imu_Calibrate(Imu* self) {
  if (!self->is_ready) return;

  printf("IMU Calibration Start (keep the robot stationary)\n");

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float sum_ax = 0.0f, sum_ay = 0.0f;
  for (uint16_t count = 0; count < IMU_CALIB_SAMPLE_COUNT;) {
    if (!Lsm6dso32_DataReady(&self->sensor)) continue;
    Lsm6dso32_Update(&self->sensor);
    sum_x += self->sensor.gyroX;
    sum_y += self->sensor.gyroY;
    sum_z += self->sensor.gyroZ;
    sum_ax += self->sensor.accelX;
    sum_ay += self->sensor.accelY;
    count++;
  }

  self->gyro_bias_x = sum_x / IMU_CALIB_SAMPLE_COUNT;
  self->gyro_bias_y = sum_y / IMU_CALIB_SAMPLE_COUNT;
  self->gyro_bias_z = sum_z / IMU_CALIB_SAMPLE_COUNT;
  // 水平方向の加速度バイアス(基板の傾き・オフセット)。TCSの対地速度推定で積分するため除去する
  self->accel_bias_x = sum_ax / IMU_CALIB_SAMPLE_COUNT;
  self->accel_bias_y = sum_ay / IMU_CALIB_SAMPLE_COUNT;

  printf("IMU Calibration Done  GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f\n",
         self->gyro_bias_x, self->gyro_bias_y, self->gyro_bias_z);
  printf("                      AccelBias[g]  X:%+.4f Y:%+.4f\n",
         self->accel_bias_x, self->accel_bias_y);
}
