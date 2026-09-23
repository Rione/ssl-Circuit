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
// 加速度残差 (a_odom - a_imu の指令加速度方向成分) の判定閾値 [m/s^2]
// (全輪が比例して空転する直進加速時のスリップは幾何残差/旋回残差に現れないため、これで検出する)
#define TCS_ACCEL_SLIP_THRESH 4.0f
// 加速度残差で判定するのは指令加速度がこれ以上のときだけ [m/s^2]
// (巡航中は車輪速度PIDのリップルで a_odom が±数m/s²揺れ、常時誤検知していたため)
#define TCS_ACCEL_DETECT_MIN 1.0f
#define TCS_VEL_SLIP_EXIT 0.10f        // スリップ解除条件: |v_odom - v_ground| がこれ未満 [m/s]
#define TCS_SLIP_HOLD_MS 50            // または、スリップ要因が消えてからこの時間経過で解除 [ms]
#define TCS_SLIP_VEL_MARGIN 0.15f      // スリップ中に許す 指令速度 - 対地速度 の上限 [m/s]
#define TCS_GROUND_VEL_KC 30.0f        // 対地速度推定の相補フィルタゲイン(グリップ中にodomへ寄せる強さ) [1/s]
#define TCS_ACCEL_LPF_HZ 15.0f         // odom加速度/IMU加速度に掛ける一次LPFのカットオフ [Hz] (位相を揃える)
#define TCS_MAX_INERTIAL_MS 300        // スリップ中にIMU積分のみで対地速度を推定する最大時間 [ms] (ドリフト防止)
#define TCS_MIN_GAIN 0.25f             // スリップ時の加速度上限比率の下限 (max_accel に対する比)
#define TCS_SLIP_GAIN_CUT 0.6f         // スリップ突入1回あたりの加速度上限比率の最大カット (×0.6 = 最大4割減)
#define TCS_RECOVERY_RATE 6.0f         // グリップ回復後の加速度上限比率の復帰速度 [1/s]
#define TCS_DEADZONE_SPEED_MPS 0.10f   // 低速デッドゾーン [m/s] (指令・実測とも未満ならスリップ判定しない)

// WheelUnitから受信した車輪角速度の1フレームあたりの最大変化 [rad/s]
// これを超える跳びは1フレームだけ保留し、次フレームで確認できなければ破棄する。
// 物理的な車輪角加速度は最大でも約600rad/s² (18m/s² / 0.03m) で1フレームあたり数rad/s。
// 実測ではフレームずれと思われる130rad/s超の単発異常値があり、スリップ誤検知の原因になっていた
#define OMNI_WHEEL_MAX_JUMP_RADPS 20.0f

#define ROBOT_KICK_INTERVAL_MS ((uint32_t)1000)  // キック間隔[ms]
#define ROBOT_KICKER_SIGNAL_INTERVAL_MS \
  ((uint32_t)100)  // チャージ/放電信号の最小送信周期[ms]

#define ROBOT_STOP_DISCHARGE_SPEED_MMPS \
  ((int16_t)100)  // 停止時にこの速度[mm/s]を超えていたら強制放電

// IMU
#define IMU_MADGWICK_BETA 0.1f  // Madgwickフィルタのゲイン(加速度補正の強さ)

// IMU取付方向: センサ-y = 機体前方(+x), センサ+x = 機体左方(+y) (z軸回りの回転なのでジャイロzはそのまま)
#define IMU_TO_ROBOT_AX(sx, sy) (-(sy))  // 機体x方向加速度
#define IMU_TO_ROBOT_AY(sx, sy) (sx)     // 機体y方向加速度
#define IMU_GRAVITY_MPS2 9.80665f        // [g] -> [m/s^2]

// 1にすると起動時(Robot_Initialize)にジャイロの静止バイアスを測定し、以降のIMU姿勢推定
// (yaw_rate/yaw_rad)に適用する。測定はブロッキングで数秒かかり、その間機体を静止させる
// 必要がある。電源を切ると測定値は失われるため毎回起動時に測定し直す仕組み。
#define IMU_CALIBRATE_ON_BOOT 1

#endif  // __PARAMMETER_H_
