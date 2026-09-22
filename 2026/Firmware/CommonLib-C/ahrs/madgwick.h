#ifndef MADGWICK_H_
#define MADGWICK_H_

#include <math.h>

// Madgwickフィルタ (IMU版: 加速度計+ジャイロのみ、磁気センサ不使用)
// ジャイロ角速度の積分に加速度計から得た重力方向でフィードバック補正をかけ、
// 姿勢クォータニオンを推定する。
// 参考: S.O.H. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", 2010
// 磁気センサが無いためヨー角に絶対方位の基準はなく、初期姿勢からの相対角として
// ドリフトを含む（ロール・ピッチは重力ベクトルで補正されるためドリフトしない）。

typedef struct {
  float beta;             // フィルタゲイン (大きいほど加速度補正が強く、収束は速いがノイズに弱い)
  float q0, q1, q2, q3;  // 姿勢クォータニオン (w,x,y,z)
} Madgwick;

static inline void Madgwick_Init(Madgwick *self, float beta) {
  self->beta = beta;
  self->q0 = 1.0f;
  self->q1 = 0.0f;
  self->q2 = 0.0f;
  self->q3 = 0.0f;
}

// gx,gy,gz: ジャイロ角速度[rad/s]
// ax,ay,az: 加速度[任意単位、内部で正規化するためスケールは問わない]
// dt: 前回呼び出しからの実経過時間[s] (呼び出し周期が一定でなくても正しく積分するため、
//     呼び出し側で実測した値を渡すこと。固定値を仮定すると、呼び出しが遅延した際に
//     その間の回転が積分されず姿勢誤差が生じる)
static inline void Madgwick_UpdateImu(Madgwick *self, float gx, float gy, float gz,
                                      float ax, float ay, float az, float dt) {
  float q0 = self->q0, q1 = self->q1, q2 = self->q2, q3 = self->q3;

  // ジャイロ角速度によるクォータニオン微分
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // 加速度が有効な場合のみ、勾配降下法による重力方向のフィードバック補正を行う
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
              _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
              _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    float norm_s = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm_s > 1e-8f) {
      norm_s = 1.0f / norm_s;
      qDot1 -= self->beta * (s0 * norm_s);
      qDot2 -= self->beta * (s1 * norm_s);
      qDot3 -= self->beta * (s2 * norm_s);
      qDot4 -= self->beta * (s3 * norm_s);
    }
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float norm_q = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  self->q0 = q0 * norm_q;
  self->q1 = q1 * norm_q;
  self->q2 = q2 * norm_q;
  self->q3 = q3 * norm_q;
}

// ヨー角[rad]をクォータニオンから抽出 (ZYXオイラー角、範囲: -π~π)
static inline float Madgwick_GetYaw(const Madgwick *self) {
  float q0 = self->q0, q1 = self->q1, q2 = self->q2, q3 = self->q3;
  return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

#endif  // MADGWICK_H_
