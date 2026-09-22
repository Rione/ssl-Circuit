// ファイル名: QuaternionProcessor.hpp
#ifndef QUATERNION_PROCESSOR_HPP
#define QUATERNION_PROCESSOR_HPP

#include <cmath>
#include <cstdint>

class QuaternionProcessor {
private:
  float q[4]; // w, x, y, z

public:
  QuaternionProcessor();
  void reset();

  // ジャイロ(rad/s)で更新
  void updateIMU(float gx, float gy, float gz, float dt);

  // ベクトル回転 (Body -> World)
  void rotateVector(float vx, float vy, float vz, float *out) const;

  // 現在のクォータニオン取得
  void getQuaternion(float *outQ) const;

  // 正規化
  void normalize();
};

#endif
