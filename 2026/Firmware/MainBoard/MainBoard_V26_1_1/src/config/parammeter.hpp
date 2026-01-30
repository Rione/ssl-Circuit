#ifndef __PARAMMETER_HPP_
#define __PARAMMETER_HPP_

namespace params {
// 機体パラメータ
constexpr float WHEEL_RADIUS = 0.03f;   // 車輪半径[m]
constexpr float ROBOT_RADIUS = 0.089f;  // ロボット半径[m]

// 制御
constexpr float CONTROL_LOOP_DT = 0.001f;  // 制御ループ周期[s]
constexpr float MAX_VEL = 3.0f;            // 最大並進速度[m/s]
constexpr float MAX_ANG_VEL = 10.0f;       // 最大角速度[rad/s]
}  // namespace params

#endif  // __PARAMMETER_HPP_