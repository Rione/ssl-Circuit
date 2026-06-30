#ifndef __PARAMMETER_H_
#define __PARAMMETER_H_

#include <stdint.h>

// 機体パラメータ
#define ROBOT_WHEEL_RADIUS 0.03f        // 車輪半径[m]
#define ROBOT_WHEEL_BASE_RADIUS 0.075f  // 車輪基底径[m]
#define ROBOT_RADIUS 0.089f             // ロボット半径[m]

extern const int16_t ROBOT_MOTOR_DEGREE[4];  // モーターの取り付け角度[deg]

// 制御
#define ROBOT_CONTROL_LOOP_DT_US 500  // 制御ループ周期[us]
#define ROBOT_MAX_VEL 3.0f            // 最大並進速度[m/s]
#define ROBOT_MAX_ANG_VEL 10.0f       // 最大角速度[rad/s]

#define ROBOT_KICK_INTERVAL_MS ((uint32_t)1000)  // キック間隔[ms]
#define ROBOT_KICKER_SIGNAL_INTERVAL_MS \
  ((uint32_t)100)  // チャージ/放電信号の最小送信周期[ms]

#endif  // __PARAMMETER_H_
