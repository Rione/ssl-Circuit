#ifndef LAT_CAP_H
#define LAT_CAP_H

// 横へ加速しているときだけのトルク上限 (ssl-RAVEN-Alula lab 2026-11-19-fw-noslip-nullfb-floor の B を、そのまま切り出したもの)。
// 2 番機・研究室の横 3.0 で、上限 3.2 V は空回りを増やさずに横の加速 (0.1〜0.8 s の平均) を 1.09 倍にした (同 lab F-235)。
// 3.6 V はそれ以上上げない (後ろの輪が 4.9 V に当たる、F-238・F-241)。
#include <math.h>

#include "parammeter.h"

// 今の周期に使うトルク上限 [V]。base: いつもの上限、lat: 横へ加速しているときの上限、
// (vx, vy): 指令の速さ、(ax, ay): S 字の参照の加速。
// lat ≤ base・止まっている (加速 0)・加速の向きが速さの向きから外れている (cos が LAT_CAP_ALIGN_COS 未満) ときは base。
// 加速の大きさが LAT_CAP_ACCEL_LO〜HI の間で 0 → 1 に効かせ、横の割合 (ay² / a²) を掛ける。
static inline float LatCapTractionLimit(float base, float lat, float vx, float vy, float ax, float ay) {
  if (lat <= base) return base;
  float a2 = ax * ax + ay * ay;
  if (a2 < 1e-6f) return base;
  // 加速が速さの向きから外れているとき (減速・回りながら進む・曲がる) は上げない。
  // 回りながら進むと機体の座標で見た速さの向きが回り続け、S 字の参照の加速が速さにほぼ直交して出続ける。
  // 以前は a·v ≥ 0 だけで判定していたので、linespin で回っている間の 41 % の時間で上限が上下した
  // (ssl-RAVEN-Alula lab 2026-11-20-track-linespin-latcap-desk、F-245)
  // 指令の速さがほぼ 0 (止まる指令) のときも上げない (a·v = 0 になり、前の判定では止まるときに上がりえた)
  const float vn = sqrtf(vx * vx + vy * vy);
  if (vn < 1e-3f) return base;
  if (ax * vx + ay * vy < LAT_CAP_ALIGN_COS * sqrtf(a2) * vn) return base;
  float w = (sqrtf(a2) - LAT_CAP_ACCEL_LO) / (LAT_CAP_ACCEL_HI - LAT_CAP_ACCEL_LO);
  if (w <= 0.0f) return base;
  if (w > 1.0f) w = 1.0f;
  return base + (lat - base) * w * (ay * ay / a2);
}

#endif  // LAT_CAP_H
