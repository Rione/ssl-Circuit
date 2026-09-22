#ifndef __PARAMMETER_H_
#define __PARAMMETER_H_

#include <stdint.h>

// 機体パラメータ
#define ROBOT_WHEEL_RADIUS 0.03f        // 車輪半径[m]
#define ROBOT_WHEEL_BASE_RADIUS 0.075f  // 車輪基底径[m]
#define ROBOT_RADIUS 0.089f             // ロボット半径[m]

extern const int16_t ROBOT_MOTOR_DEGREE[4];  // モーターの取り付け角度[deg]

// 制御
#define ROBOT_CONTROL_LOOP_DT_US 1000  // 制御ループ周期[us]
#define ROBOT_MAX_VEL 3.0f             // 最大並進速度[m/s]
#define ROBOT_MAX_ANG_VEL 10.0f        // 最大角速度[rad/s]

// トラクションコントロール (TCS) パラメータ
#define TCS_ENABLE 1                   // TCS有効化フラグ (1: 有効, 0: 無効)
#define TCS_ENABLE_S_CURVE 1           // S字/ジャーク制限加減速有効化 (1: 有効, 0: 無効)
#define TCS_MAX_ACCEL 18.0f            // 最大並進加速度 [m/s^2] (摩擦限界を考慮: 約1.8G)
#define TCS_MAX_JERK 250.0f            // 最大並進ジャーク [m/s^3] (俊敏な応答とスパイク低減の両立)
#define TCS_MAX_ANG_ACCEL 60.0f        // 最大角加速度 [rad/s^2]
#define TCS_MAX_ANG_JERK 600.0f        // 最大角ジャーク [rad/s^3]
#define TCS_GEOM_K 1.158456f           // 4輪幾何拘束係数: sqrt(2)*sin(55deg)
#define TCS_GEOM_SLIP_THRESH 15.0f     // 幾何残差スリップ判定閾値 [rad/s] (定常公差残差による誤検知防止)
#define TCS_ROT_SLIP_THRESH 4.5f       // 旋回ジャイロ残差スリップ判定閾値 [rad/s]
#define TCS_SLIP_GAIN 0.6f             // スリップ検出時の抑制ゲイン
#define TCS_MIN_GAIN 0.70f             // 最小抑制ゲイン (出力制限の下限比率: 70%にとどめて失速防止)
#define TCS_RECOVERY_RATE 6.0f         // グリップ回復後のゲイン復帰速度 [1/s] (約0.05秒で素早く復帰)
#define TCS_DEADZONE_SPEED_MPS 0.25f   // 低速デッドゾーン [m/s] (これ未満の極低速ではスリップ介入をバイパス)
#define TCS_NOMINAL_VOLTAGE 16.0f      // 基準バッテリー電圧 [V] (4S LiPo想定)

#define ROBOT_KICK_INTERVAL_MS ((uint32_t)1000)  // キック間隔[ms]
#define ROBOT_KICKER_SIGNAL_INTERVAL_MS \
  ((uint32_t)100)  // チャージ/放電信号の最小送信周期[ms]

#define ROBOT_STOP_DISCHARGE_SPEED_MMPS \
  ((int16_t)100)  // 停止時にこの速度[mm/s]を超えていたら強制放電

// IMU
#define IMU_MADGWICK_BETA 0.1f  // Madgwickフィルタのゲイン(加速度補正の強さ)

// 1にすると起動時(Robot_Initialize)にジャイロの静止バイアスを測定し、以降のIMU姿勢推定
// (yaw_rate/yaw_rad)に適用する。測定はブロッキングで数秒かかり、その間機体を静止させる
// 必要がある。電源を切ると測定値は失われるため毎回起動時に測定し直す仕組み。
#define IMU_CALIBRATE_ON_BOOT 1

#endif  // __PARAMMETER_H_
