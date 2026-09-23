#include "tcs_log.h"

#include <stdio.h>

// int16 固定小数で保持 (1サンプル62byte x 1000 = 約62KB)
typedef struct {
  uint16_t t_ms;         // テスト開始からの時間 [ms]
  uint8_t tcs_on;        // 1: TCS有り, 0: TCS無し
  uint8_t slip;          // bit0-2: TCS_SLIP_* / bit6: 電圧制御のトルク上限が効いた / bit7: is_slipping
  int16_t target_vx;     // 目標速度X [mm/s]
  int16_t cmd_vx;        // TCS通過後の指令速度X [mm/s]
  int16_t odom_vx;       // オドメトリ速度 [mm/s]
  int16_t odom_vy;
  int16_t ground_vx;     // 推定対地速度 [mm/s]
  int16_t ground_vy;
  int16_t a_odom_x;      // LPF済みオドメトリ加速度X [cm/s^2]
  int16_t a_imu_x;       // LPF済みIMU加速度X [cm/s^2]
  int16_t accel_res;     // 加速度残差 [cm/s^2]
  int16_t geom_res;      // 幾何残差 [0.1rad/s]
  int16_t rot_res;       // 旋回残差 [mrad/s]
  int16_t accel_gain;    // 加速度上限比率 [x1000]
  int16_t wheel[4];      // 実測車輪角速度 [0.01rad/s]
  // 横ずれの切り分け用 (指令側か、WheelUnitの追従誤差か、スリップか)
  int16_t cmd_vy;        // TCS通過後の指令速度Y [mm/s]
  int16_t cmd_w;         // TCS通過後の指令角速度 (ヘディング制御の出力) [mrad/s]
  int16_t gyro_w;        // IMUヨーレート [mrad/s]
  int16_t target_wheel[4];  // 目標車輪角速度 [0.01rad/s] (wheel[] と直接比較できる)
  // WheelUnitからの受信の健全性 (途絶えている間の wheel[] は古い値のまま)
  uint8_t rx_stall;      // bit i: 輪 i の受信が途絶えて再開を試みている最中
  uint16_t rx_restart;   // 記録開始からの受信再開回数 (4輪合計)
  int16_t volt[4];       // 印加電圧 [0.01V] (電圧制御中のみ。速度モード中は0)
} TcsLogSample;

static TcsLogSample samples[TCS_LOG_MAX_SAMPLES];
static uint16_t sample_count = 0;
static uint16_t dump_index = 0;
static uint16_t rx_restart_base = 0;  // 記録開始時の受信再開回数 (4輪合計)

static uint16_t SumRxRestart(const OmniDrive* omni_drive) {
  uint16_t sum = 0;
  for (int i = 0; i < 4; i++) sum += omni_drive->wheel_rx_restart_count[i];
  return sum;
}

static int16_t ToI16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}

void TcsLog_Reset(void) {
  sample_count = 0;
  dump_index = 0;
}

void TcsLog_Record(uint32_t t_ms, bool tcs_on, int16_t target_vx_mmps, float gyro_yaw_rate,
                   const OmniDrive* omni_drive) {
  if (sample_count >= TCS_LOG_MAX_SAMPLES) return;

  const TractionControl* tcs = &omni_drive->tcs;
  if (sample_count == 0) rx_restart_base = SumRxRestart(omni_drive);
  TcsLogSample* s = &samples[sample_count++];
  s->t_ms = (uint16_t)t_ms;
  s->tcs_on = tcs_on ? 1 : 0;
  s->slip = (uint8_t)(tcs->slip_flags | (tcs->is_slipping ? 0x80U : 0x00U) |
                      (omni_drive->volt_traction_limited ? 0x40U : 0x00U));
  s->target_vx = target_vx_mmps;
  s->cmd_vx = ToI16(tcs->current_vx * 1000.0f);
  s->odom_vx = ToI16(tcs->odom_vx * 1000.0f);
  s->odom_vy = ToI16(tcs->odom_vy * 1000.0f);
  s->ground_vx = ToI16(tcs->ground_vx * 1000.0f);
  s->ground_vy = ToI16(tcs->ground_vy * 1000.0f);
  s->a_odom_x = ToI16(tcs->a_odom_x * 100.0f);
  s->a_imu_x = ToI16(tcs->a_imu_x * 100.0f);
  s->accel_res = ToI16(tcs->accel_residual * 100.0f);
  s->geom_res = ToI16(tcs->geom_residual * 10.0f);
  s->rot_res = ToI16(tcs->rot_residual * 1000.0f);
  s->accel_gain = ToI16(tcs->accel_gain * 1000.0f);
  for (int i = 0; i < 4; i++) {
    s->wheel[i] = ToI16(omni_drive->vel_wheel_angular[i] * 100.0f);
  }
  s->cmd_vy = ToI16(tcs->current_vy * 1000.0f);
  s->cmd_w = ToI16(tcs->current_omega * 1000.0f);
  s->gyro_w = ToI16(gyro_yaw_rate * 1000.0f);
  for (int i = 0; i < 4; i++) {
    s->target_wheel[i] = ToI16(omni_drive->target_wheel_angular[i] * 100.0f);
  }
  s->rx_stall = 0;
  for (int i = 0; i < 4; i++) {
    if (omni_drive->wheel_rx_stalled[i]) s->rx_stall |= (uint8_t)(1U << i);
  }
  s->rx_restart = (uint16_t)(SumRxRestart(omni_drive) - rx_restart_base);
  for (int i = 0; i < 4; i++) {
    s->volt[i] = ToI16(omni_drive->cmd_voltage[i] * 100.0f);
  }
}

bool TcsLog_DumpStep(void) {
  // 0: パラメータ行, 1: ヘッダ行, 2〜: データ行
  if (dump_index == 0) {
    printf("# TCS log: max_accel=%d jerk=%d accel_th=%d geom_th=%d rot_th=%d [x100] "
           "margin=%d exit=%d [mm/s] n=%u\n",
           (int)(TCS_MAX_ACCEL * 100), (int)(TCS_MAX_JERK * 100),
           (int)(TCS_ACCEL_SLIP_THRESH * 100), (int)(TCS_GEOM_SLIP_THRESH * 100),
           (int)(TCS_ROT_SLIP_THRESH * 100), (int)(TCS_SLIP_VEL_MARGIN * 1000),
           (int)(TCS_VEL_SLIP_EXIT * 1000), sample_count);
  } else if (dump_index == 1) {
    // ヘッダは約250文字あり、1回で送ると _write (main.c) の送信タイムアウト10ms
    // (250kbpsで約250文字分) に掛かって途中で切れるため、分けて送る (CSV上は1行)
    fputs("t_ms,tcs_on,slip,target_vx,cmd_vx,odom_vx,odom_vy,ground_vx,ground_vy,", stdout);
    fflush(stdout);
    fputs("a_odom_x_cm,a_imu_x_cm,accel_res_cm,geom_res_x10,rot_res_mrad,accel_gain_x1000,",
          stdout);
    fflush(stdout);
    fputs("w0_x100,w1_x100,w2_x100,w3_x100,cmd_vy,cmd_w_mrad,gyro_mrad,", stdout);
    fflush(stdout);
    fputs("t0_x100,t1_x100,t2_x100,t3_x100,rx_stall,rx_restart,v0_x100,v1_x100,v2_x100,v3_x100\n",
          stdout);
  } else if (dump_index - 2 < sample_count) {
    const TcsLogSample* s = &samples[dump_index - 2];
    printf("%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,"
           "%d,%d,%d,%d\n",
           s->t_ms, s->tcs_on, s->slip, s->target_vx, s->cmd_vx, s->odom_vx, s->odom_vy,
           s->ground_vx, s->ground_vy, s->a_odom_x, s->a_imu_x, s->accel_res, s->geom_res,
           s->rot_res, s->accel_gain, s->wheel[0], s->wheel[1], s->wheel[2], s->wheel[3],
           s->cmd_vy, s->cmd_w, s->gyro_w, s->target_wheel[0], s->target_wheel[1],
           s->target_wheel[2], s->target_wheel[3], s->rx_stall, s->rx_restart, s->volt[0],
           s->volt[1], s->volt[2], s->volt[3]);
  } else {
    printf("# TCS log end\n");
    return true;
  }
  dump_index++;
  return false;
}
