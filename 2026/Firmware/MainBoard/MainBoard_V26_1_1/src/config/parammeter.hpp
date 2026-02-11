#ifndef __PARAMMETER_HPP_
#define __PARAMMETER_HPP_

#include <stdint.h>

namespace robot_params {
// 機体パラメータ
constexpr float kWheelRadius = 0.03f;                      // 車輪半径[m]
constexpr float kWheelBaseRadius = 0.075f;                 // 車輪基底径[m]
constexpr float kRobotRadius = 0.089f;                     // ロボット半径[m]
constexpr int16_t kMotorDegree[4] = {55, 135, -135, -55};  // モーターの取り付け角度[deg]

// 制御
constexpr float kControlLoopDt = 0.001f;  // 制御ループ周期[s]
constexpr float kMaxVel = 3.0f;           // 最大並進速度[m/s]
constexpr float kMaxAngVel = 10.0f;       // 最大角速度[rad/s]

constexpr uint16_t kKickIntervalMs = 500;  // キック間隔[ms]
}  // namespace robot_params

#endif  // __PARAMMETER_HPP_