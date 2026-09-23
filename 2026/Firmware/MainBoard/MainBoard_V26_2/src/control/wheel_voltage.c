#include "wheel_voltage.h"

#include <stdbool.h>

#include "mymath.h"
#include "parammeter.h"

// 車輪を浮かせて一定電圧をかけたときの定常回転数 [rad/s] (大きさ)。2026-09-23 の測り直し
// (docs/data/voltage_eval_20260923_recal.csv)。目標角速度からこの表を逆に引いて電圧を決める。
// 回転数は電圧に比例しない (逆転は電圧が高いほど伸びが鈍い) ので、直線近似 V = Ke·ω + Vf ではなく
// 表の間を直線でつなぐ。正転と逆転で最大4割違うのは、エンコーダのオフセット誤差と大きな進角 (K_ADV)
// によるとみられる。WheelUnit のキャリブレーションをやり直したら測り直して更新すること
#define WHEEL_VOLT_TABLE_N 6
static const float kTableVolt[WHEEL_VOLT_TABLE_N] = {0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f};
static const float kTableOmegaPos[4][WHEEL_VOLT_TABLE_N] = {
    {15.35f, 31.47f, 48.18f, 65.67f, 101.28f, 136.82f},
    {15.16f, 30.70f, 47.50f, 64.90f, 100.90f, 138.32f},
    {14.86f, 30.38f, 47.31f, 64.75f, 101.43f, 139.38f},
    {15.07f, 31.64f, 47.96f, 64.25f, 97.07f, 128.63f},
};
static const float kTableOmegaNeg[4][WHEEL_VOLT_TABLE_N] = {
    {14.84f, 29.31f, 43.55f, 57.32f, 83.20f, 107.36f},
    {14.53f, 28.52f, 42.44f, 56.24f, 82.07f, 105.98f},
    {14.22f, 28.01f, 41.64f, 54.69f, 78.83f, 101.48f},
    {15.38f, 31.36f, 47.05f, 62.40f, 92.76f, 121.96f},
};

// 床の上の負荷分 [V] (浮かせた表に対して床で足りなかった分)。2026-09-23 の床のTCSテスト
// (docs/data/tcs_voltage_ff_20260923.csv) に V − V_ss(ω) = Ka·α + Vd·sign(ω) を当てはめた Vd
static const float kLoadVolt[4] = {0.48f, 0.53f, 0.34f, 0.52f};
#define WHEEL_VOLT_LOAD_SIGN_WIDTH 2.0f  // 負荷分の符号を滑らかにする幅 [rad/s] (0付近のばたつき防止)

// 回転数の大きさ omega_abs を出すのに要る電圧の大きさ (原点と表の点を直線でつなぐ。表の外は最後の傾きで延長)
static float WheelVoltage_Lookup(const float* table, float omega_abs) {
  float prev_omega = 0.0f, prev_volt = 0.0f;
  for (int i = 0; i < WHEEL_VOLT_TABLE_N; i++) {
    if (omega_abs <= table[i] || i == WHEEL_VOLT_TABLE_N - 1) {
      float slope = (kTableVolt[i] - prev_volt) / (table[i] - prev_omega);
      return prev_volt + slope * (omega_abs - prev_omega);
    }
    prev_omega = table[i];
    prev_volt = kTableVolt[i];
  }
  return 0.0f;  // 到達しない
}

float WheelVoltage_Feedforward(int wheel, float omega_ref, float alpha_ref) {
  bool is_pos = omega_ref >= 0.0f;
  const float* table = is_pos ? kTableOmegaPos[wheel] : kTableOmegaNeg[wheel];
  float volt = WheelVoltage_Lookup(table, is_pos ? omega_ref : -omega_ref);
  if (!is_pos) volt = -volt;
#if WHEEL_VOLT_USE_LOAD_FF
  volt += kLoadVolt[wheel] * Constrain(omega_ref / WHEEL_VOLT_LOAD_SIGN_WIDTH, -1.0f, 1.0f);
#endif
  volt += WHEEL_VOLT_KA * alpha_ref;
  return Constrain(volt, -WHEEL_VOLT_MAX, WHEEL_VOLT_MAX);
}
