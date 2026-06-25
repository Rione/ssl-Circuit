// ファイル名: QuaternionProcessor.cpp
#include "QuaternionProcessor.hpp"
#include <cmath> // sqrt, sin, cos

QuaternionProcessor::QuaternionProcessor() { reset(); }

void QuaternionProcessor::reset() {
  q[0] = 1.0f;
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = 0.0f;
}

void QuaternionProcessor::updateIMU(float gx, float gy, float gz, float dt) {
  float norm = std::sqrt(gx * gx + gy * gy + gz * gz);
  // 回転が極小の場合は更新しない（ゼロ除算防止）
  if (norm < 1e-6f)
    return;

  // dq/dt = 0.5 * q * omega
  float halfTheta = norm * dt * 0.5f;
  float sinHalfTheta = std::sin(halfTheta);
  float cosHalfTheta = std::cos(halfTheta);

  float qw = cosHalfTheta;
  float qx = (gx / norm) * sinHalfTheta;
  float qy = (gy / norm) * sinHalfTheta;
  float qz = (gz / norm) * sinHalfTheta;

  // クォータニオン乗算
  float w = q[0], x = q[1], y = q[2], z = q[3];
  q[0] = w * qw - x * qx - y * qy - z * qz;
  q[1] = w * qx + x * qw + y * qz - z * qy;
  q[2] = w * qy - x * qz + y * qw + z * qx;
  q[3] = w * qz + x * qy - y * qx + z * qw;

  normalize();
}

void QuaternionProcessor::normalize() {
  float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f) {
    float invN = 1.0f / n;
    q[0] *= invN;
    q[1] *= invN;
    q[2] *= invN;
    q[3] *= invN;
  }
}

void QuaternionProcessor::rotateVector(float vx, float vy, float vz,
                                       float *out) const {
  // クォータニオンによるベクトル回転: v' = q * v * q_conjugate
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float x2 = x * x, y2 = y * y, z2 = z * z;
  float xy = x * y, xz = x * z, yz = y * z;
  float wx = w * x, wy = w * y, wz = w * z;

  // Rotation Matrix
  float r00 = 1.0f - 2.0f * (y2 + z2);
  float r01 = 2.0f * (xy - wz);
  float r02 = 2.0f * (xz + wy);

  float r10 = 2.0f * (xy + wz);
  float r11 = 1.0f - 2.0f * (x2 + z2);
  float r12 = 2.0f * (yz - wx);

  float r20 = 2.0f * (xz - wy);
  float r21 = 2.0f * (yz + wx);
  float r22 = 1.0f - 2.0f * (x2 + y2);

  out[0] = r00 * vx + r01 * vy + r02 * vz;
  out[1] = r10 * vx + r11 * vy + r12 * vz;
  out[2] = r20 * vx + r21 * vy + r22 * vz;
}

void QuaternionProcessor::getQuaternion(float *outQ) const {
  outQ[0] = q[0];
  outQ[1] = q[1];
  outQ[2] = q[2];
  outQ[3] = q[3];
}
