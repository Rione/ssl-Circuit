#include "RobotPoseEstimator.hpp"
#include <cmath>

RobotPoseEstimator::RobotPoseEstimator()
    : x(4, 1), P(4, 4), F(4, 4), Q(4, 4), H(2, 4), R(4, 4), qProc(),
    //H(4,4) R(4,4)　このR何？
      accelNoiseStdDev(0.5f), encNoiseStdDev(0.1f) {
  offsetAccel[0] = offsetAccel[1] = offsetAccel[2] = 0.0f;
  offsetGyro[0] = offsetGyro[1] = offsetGyro[2] = 0.0f;

  // 初期化: 単位行列など
  P = SimpleMatrix::identity(4);
  F = SimpleMatrix::identity(4);
  //H = SimpleMatrix::identity(2); //(4)
  H.setZero();
  R.setZero();
  // 状態: px, py, vx, vy
  x.setZero();
}

void RobotPoseEstimator::init(float aNoise, float eNoise) {
  accelNoiseStdDev = aNoise;
  encNoiseStdDev = eNoise;

  // ノイズ行列の設定 (簡易)
  Q.setZero();
  for (int i = 0; i < 4; i++)
    Q.set(i, i, 0.01f); // プロセスノイズ

  R.setZero(); // 観測ノイズ (updateで設定)
}

void RobotPoseEstimator::setOffsets(float ax, float ay, float az, float gx,
                                    float gy, float gz) {
  offsetAccel[0] = ax;
  offsetAccel[1] = ay;
  offsetAccel[2] = az;
  offsetGyro[0] = gx;
  offsetGyro[1] = gy;
  offsetGyro[2] = gz;
}

void RobotPoseEstimator::update(float ax, float ay, float az, float gx,
                                float gy, float gz, float encVx, float encVy,
                                float dt) {
  // IMU姿勢更新

  qProc.updateIMU(gx, gy, gz, dt);
  // 加速度を世界座標系へ変換
  float accBody[3] = {ax, ay, az};
  float accWorld[3];
  qProc.rotateVector(accBody[0], accBody[1], accBody[2], accWorld);

  // Z軸重力除去 (簡易的: Zが上向きと仮定)
  // accWorld[2] -= 9.8f;

  // EKF 予測ステップ
  // F行列:
  // px' = px + vx*dt
  // py' = py + vy*dt
  // vx' = vx
  // vy' = vy
  F.set(0, 2, dt);
  F.set(1, 3, dt);

  SimpleMatrix x_pred = F * x;
  SimpleMatrix P_pred = F * P * F.transpose() + Q;

  // EKF 更新ステップ

  // 簡易実装: 入力u = accWorld として予測ステップに加える
  x_pred.set(2, 0, x_pred.get(2, 0) + accWorld[0] * dt);
  x_pred.set(3, 0, x_pred.get(3, 0) + accWorld[1] * dt);

  // エンコーダはロボット座標系。姿勢qを使ってWorldへ変換
  float encV_Body[3] = {encVx, encVy, 0};
  float encV_World[3];
  qProc.rotateVector(encV_Body[0], encV_Body[1], encV_Body[2], encV_World);

  // 観測ベクトル z
  SimpleMatrix z(2, 1);
  z.set(0, 0, encV_World[0]);
  z.set(1, 0, encV_World[1]);

  // 観測行列 H (vx, vyを観測)
  // z = [vx, vy]T
  // x = [px, py, vx, vy]T
  H.setZero();
  H.set(0, 2, 1.0f); //(1,3)に1
  H.set(1, 3, 1.0f); //(2,4)に1

  // 観測ノイズ R
  SimpleMatrix R_local(2, 2);
  R_local.setZero();
  R_local.set(0, 0, encNoiseStdDev * encNoiseStdDev);
  R_local.set(1, 1, encNoiseStdDev * encNoiseStdDev);

  // カルマンゲイン K = P*H^T * (H*P*H^T + R)^-1
  SimpleMatrix Ht = H.transpose(); //観測行列
  SimpleMatrix S = H * P_pred * Ht + R_local; //前のequationだと4*4＋2*2だから計算不可。　
  SimpleMatrix K = P_pred * Ht * S.inverse2x2(); // 2x2逆行列

  // 更新
  SimpleMatrix y = z - H * x_pred;
  x = x_pred + K * y;
  P = (SimpleMatrix::identity(4) - K * H) * P_pred;
}

float RobotPoseEstimator::getPosX() const { return x.get(0, 0); }
float RobotPoseEstimator::getPosY() const { return x.get(1, 0); }
float RobotPoseEstimator::getVelX() const { return x.get(2, 0); }
float RobotPoseEstimator::getVelY() const { return x.get(3, 0); }

void RobotPoseEstimator::getQuaternion(float *qOut) {
  qProc.getQuaternion(qOut);
}
