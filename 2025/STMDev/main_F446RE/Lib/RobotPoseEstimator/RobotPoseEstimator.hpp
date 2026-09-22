#ifndef ROBOT_POSE_ESTIMATOR_HPP
#define ROBOT_POSE_ESTIMATOR_HPP

#include "QuaternionProcessor.hpp"
#include "SimpleMatrix.hpp"
#include <cstdint>

class RobotPoseEstimator {
private:
  // 状態変数: [px, py, vx, vy]T (4x1)
  SimpleMatrix x;
  SimpleMatrix P;
  SimpleMatrix F; // 状態遷移行列
  SimpleMatrix Q; // プロセスノイズ行列
  SimpleMatrix H; // 観測行列
  SimpleMatrix R; // 観測ノイズ行列

  QuaternionProcessor qProc;

  // パラメータ
  float accelNoiseStdDev;
  float encNoiseStdDev;

  // オフセット
  float offsetAccel[3];
  float offsetGyro[3];

public:
  RobotPoseEstimator();

  // 初期化 (パラメータ設定)
  void init(float aNoise, float eNoise);

  // キャリブレーション値をセット
  void setOffsets(float ax, float ay, float az, float gx, float gy, float gz);

  /**
   * メイン更新関数
   * @param ax, ay, az : 加速度センサ生データ (m/s^2)
   * @param gx, gy, gz : ジャイロセンサ生データ (rad/s)
   * @param encVx, encVy : エンコーダ由来のロボットローカル速度 (m/s)
   * @param dt : 前回の呼び出しからの経過時間 (秒)
   */
  void update(float ax, float ay, float az, float gx, float gy, float gz,
              float encVx, float encVy, float dt);

  // ゲッター
  float getPosX() const;
  float getPosY() const;
  float getVelX() const;
  float getVelY() const;

  // クォータニオン取得
  void getQuaternion(float *qOut);
};

#endif
