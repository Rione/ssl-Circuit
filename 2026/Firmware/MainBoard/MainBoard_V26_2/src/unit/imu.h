#ifndef __IMU_H_
#define __IMU_H_

#include <stdint.h>

#include "lsm6dso32.h"
#include "madgwick.h"
#include "timer.h"

#define IMU_CALIB_SAMPLE_COUNT 200U  // ジャイロキャリブレーションのサンプル数(ODR104Hzで約2秒)
// センサ異常でDRDYが立たない場合に永久ループしないためのタイムアウト[ms]。
// 通常は約2秒で完了するため、十分な余裕を持たせている。
#define IMU_CALIB_TIMEOUT_MS ((uint32_t)5000)
// Madgwick積分1ステップあたりのdt上限[s]。呼び出しが何らかの理由で遅延した場合に
// 一度の積分で過大な回転を適用してしまうのを防ぐための安全弁(通常はODR104Hzの
// 周期=約9.6msで呼ばれる想定)。
#define IMU_MAX_DT_S 0.05f

typedef struct {
  Lsm6dso32 sensor;
  Madgwick ahrs;
  Timer update_timer;  // Imu_Update呼び出し間隔の実測用(固定dtを仮定しないため)
  uint8_t is_ready;    // WHO_AM_I確認済みか

  float gyro_bias_x, gyro_bias_y, gyro_bias_z;  // [dps] Imu_Calibrateで測定した静止バイアス

  float accel_x, accel_y;  // [g]
  float yaw_rate;          // [rad/s] (ジャイロZ、機体旋回方向の角速度、バイアス補正済み)
  float yaw_rad;            // [rad] Madgwickフィルタによる姿勢推定 (-π~π、起動時を基準とした相対角)
} Imu;

void Imu_Init(Imu *self);
void Imu_Update(Imu *self);

// ジャイロの静止バイアスを測定する(ブロッキング、呼び出し中は機体を静止させること)
void Imu_Calibrate(Imu *self);

#endif  // __IMU_H_
