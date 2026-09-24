#ifndef __IMU_H_
#define __IMU_H_

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dso32.h"
#include "madgwick.h"
#include "timer.h"

#define IMU_CALIB_SAMPLE_COUNT 200U  // ジャイロキャリブレーションのサンプル数(ODR104Hzで約2秒)
#define IMU_CALIB_SETTLE_MS 500U     // キャリブレーション開始前に読み捨てる時間 [ms] (ジャイロ起動直後の外れ値対策)
#define IMU_CALIB_MAX_STD_DPS 0.3f   // 測定中のジャイロ標準偏差がこれ以下なら静止とみなす [dps]
#define IMU_CALIB_MAX_TRY 5U         // 静止と判定できるまで測り直す最大回数
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
  float accel_bias_x, accel_bias_y;             // [g] Imu_Calibrateで測定した静止バイアス(センサ座標)

  float accel_x, accel_y;  // [g] センサ座標・バイアス未補正 (Rock5A送信用)
  float accel_robot_x, accel_robot_y;  // [m/s^2] 機体座標(x:前, y:左)・バイアス補正済み (TCS用)
  float yaw_rate;          // [rad/s] (ジャイロZ、機体旋回方向の角速度、バイアス補正済み)
  float yaw_rad;            // [rad] Madgwickフィルタによる姿勢推定 (-π~π、起動時を基準とした相対角)
} Imu;

void Imu_Init(Imu *self);
void Imu_Update(Imu *self);

// ジャイロ・加速度xyの静止バイアスを測定する(ブロッキング、呼び出し中は機体を静止させること)。
// 静止と判定できれば true。できなかった場合も最もばらつきの小さい測定値を適用する
bool Imu_Calibrate(Imu *self);
// 現在のバイアス値をフラッシュに保存する (セクタ消去で1〜2秒止まるため起動時のみ)
bool Imu_SaveCalibration(const Imu *self);
// フラッシュに保存されたバイアス値を読み込む。未保存・破損なら false (値は変えない)
bool Imu_LoadCalibration(Imu *self);

#endif  // __IMU_H_
