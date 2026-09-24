#include "imu.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "flash.h"
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

// 1回分の測定: ジャイロ3軸と加速度xyをIMU_CALIB_SAMPLE_COUNT回サンプリングして平均する。
// センサ異常でDRDYが立たない場合に永久ループしないよう、IMU_CALIB_TIMEOUT_MSで打ち切る
// (打ち切ったときは実際に集まったサンプル数で平均する)。
// 戻り値はジャイロ3軸の標準偏差の最大値 [dps] (測定中に機体が動いていないかの判定用)
static float Imu_MeasureBias(Imu* self, float gyro_bias[3], float accel_bias[2]) {
  float sum[3] = {0.0f, 0.0f, 0.0f};
  float sum_sq[3] = {0.0f, 0.0f, 0.0f};
  float sum_a[2] = {0.0f, 0.0f};
  uint32_t start_tick = HAL_GetTick();
  uint16_t count = 0;
  while (count < IMU_CALIB_SAMPLE_COUNT) {
    // 加速度だけの更新で読むと同じジャイロ値を二重に数えるため、ジャイロの更新を待つ
    if (!Lsm6dso32_GyroDataReady(&self->sensor)) {
      if ((HAL_GetTick() - start_tick) > IMU_CALIB_TIMEOUT_MS) {
        printf("IMU Calibration Timeout (DRDY not ready), using %u samples\n", count);
        break;
      }
      continue;
    }
    Lsm6dso32_Update(&self->sensor);
    const float g[3] = {self->sensor.gyroX, self->sensor.gyroY, self->sensor.gyroZ};
    for (int i = 0; i < 3; i++) {
      sum[i] += g[i];
      sum_sq[i] += g[i] * g[i];
    }
    sum_a[0] += self->sensor.accelX;
    sum_a[1] += self->sensor.accelY;
    count++;
  }

  if (count == 0) {
    for (int i = 0; i < 3; i++) gyro_bias[i] = 0.0f;
    for (int i = 0; i < 2; i++) accel_bias[i] = 0.0f;
    return 999.0f;  // タイムアウトで1件も取れなければ「静止」判定させない大きな値を返す
  }

  float max_std = 0.0f;
  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = sum[i] / count;
    float var = sum_sq[i] / count - gyro_bias[i] * gyro_bias[i];
    float std = sqrtf(var > 0.0f ? var : 0.0f);
    if (std > max_std) max_std = std;
  }
  for (int i = 0; i < 2; i++) {
    accel_bias[i] = sum_a[i] / count;
  }
  return max_std;
}

// ジャイロ・加速度xyの静止バイアスを測定する(ブロッキング)
bool Imu_Calibrate(Imu* self) {
  if (!self->is_ready) return false;

  printf("IMU Calibration Start (keep the robot stationary)\n");

  // ジャイロは有効化の直後しばらく出力が安定しない。実測では起動直後の外れ値が平均に混ざり、
  // 測定中の標準偏差が3〜4dps (静止時のノイズは約0.1dps)、バイアスが毎回0.05〜0.35dpsずれて
  // ヨー角が数十°/分ずれていた。最初の IMU_CALIB_SETTLE_MS 分は読み捨てる
  uint32_t settle_start = HAL_GetTick();
  while (HAL_GetTick() - settle_start < IMU_CALIB_SETTLE_MS) {
    if (Lsm6dso32_DataReady(&self->sensor)) Lsm6dso32_Update(&self->sensor);
  }

  // 標準偏差が大きい (機体が動いた) 場合は測り直し、最もばらつきの小さい結果を採用する
  float best_gyro[3] = {0.0f, 0.0f, 0.0f};
  float best_accel[2] = {0.0f, 0.0f};
  float best_std = 0.0f;
  bool is_stationary = false;
  for (uint8_t attempt = 1; attempt <= IMU_CALIB_MAX_TRY; attempt++) {
    float gyro[3], accel[2];
    float std = Imu_MeasureBias(self, gyro, accel);
    printf("  try %u: GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f  std[dps] %.3f\n", attempt, gyro[0],
           gyro[1], gyro[2], std);
    if (attempt == 1 || std < best_std) {
      best_std = std;
      for (int i = 0; i < 3; i++) best_gyro[i] = gyro[i];
      for (int i = 0; i < 2; i++) best_accel[i] = accel[i];
    }
    if (std <= IMU_CALIB_MAX_STD_DPS) {
      is_stationary = true;
      break;
    }
  }

  self->gyro_bias_x = best_gyro[0];
  self->gyro_bias_y = best_gyro[1];
  self->gyro_bias_z = best_gyro[2];
  // 水平方向の加速度バイアス(基板の傾き・オフセット)。TCSの対地速度推定で積分するため除去する
  self->accel_bias_x = best_accel[0];
  self->accel_bias_y = best_accel[1];

  printf("IMU Calibration %s  GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f  std[dps] %.3f\n",
         is_stationary ? "Done" : "NG (robot was moving)", self->gyro_bias_x,
         self->gyro_bias_y, self->gyro_bias_z, best_std);
  printf("                      AccelBias[g]  X:%+.4f Y:%+.4f\n", self->accel_bias_x,
         self->accel_bias_y);
  return is_stationary;
}

// フラッシュに保存するキャリブレーション値 (CommonLib-C/flash/flash.h のユーザー領域)
typedef struct {
  uint32_t magic;
  float gyro_bias[3];   // [dps]
  float accel_bias[2];  // [g]
  uint32_t checksum;
} ImuCalibData;

#define IMU_CALIB_MAGIC 0x494D5531U  // "IMU1"

static uint32_t Imu_CalibChecksum(const ImuCalibData* data) {
  const uint32_t* words = (const uint32_t*)data;
  uint32_t sum = 0x5A5A5A5AU;
  for (size_t i = 0; i < offsetof(ImuCalibData, checksum) / sizeof(uint32_t); i++) {
    sum = ((sum << 1) | (sum >> 31)) ^ words[i];
  }
  return sum;
}

bool Imu_SaveCalibration(const Imu* self) {
  ImuCalibData data = {
      .magic = IMU_CALIB_MAGIC,
      .gyro_bias = {self->gyro_bias_x, self->gyro_bias_y, self->gyro_bias_z},
      .accel_bias = {self->accel_bias_x, self->accel_bias_y},
  };
  data.checksum = Imu_CalibChecksum(&data);
  // F446 はセクタ7 (128KB) を丸ごと消去するため1〜2秒ほど止まる (起動時のみ呼ぶこと)
  if (Flash_WriteData(FLASH_USER_START_ADDR, &data, sizeof(data)) != HAL_OK) {
    printf("IMU Calibration save failed\n");
    return false;
  }
  printf("IMU Calibration saved to flash\n");
  return true;
}

bool Imu_LoadCalibration(Imu* self) {
  ImuCalibData data;
  Flash_ReadData(FLASH_USER_START_ADDR, &data, sizeof(data));
  if (data.magic != IMU_CALIB_MAGIC || data.checksum != Imu_CalibChecksum(&data)) {
    return false;
  }
  self->gyro_bias_x = data.gyro_bias[0];
  self->gyro_bias_y = data.gyro_bias[1];
  self->gyro_bias_z = data.gyro_bias[2];
  self->accel_bias_x = data.accel_bias[0];
  self->accel_bias_y = data.accel_bias[1];
  printf("IMU Calibration loaded from flash  GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f\n",
         self->gyro_bias_x, self->gyro_bias_y, self->gyro_bias_z);
  printf("                      AccelBias[g]  X:%+.4f Y:%+.4f\n", self->accel_bias_x,
         self->accel_bias_y);
  return true;
}
