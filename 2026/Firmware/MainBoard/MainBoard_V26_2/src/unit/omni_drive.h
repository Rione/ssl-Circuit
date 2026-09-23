#ifndef __OMNI_DRIVE_H_
#define __OMNI_DRIVE_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"
#include "maf.h"
#include "mymath.h"
#include "parammeter.h"
#include "serial.h"
#include "timer.h"
#include "traction_control.h"

typedef struct {
  int16_t vel_x;
  int16_t vel_y;
  int16_t motor_angular_velocity[4];
} OmniDriveStatus;

typedef struct {
  Serial* serials[4];
  float vel_wheel_angular[4];     // WheelUnitから受信した実測車輪角速度 [rad/s]
  float target_wheel_angular[4];  // 直近に送信した目標車輪角速度 [rad/s] (クランプ前)
  uint8_t emg;
  uint8_t ready;
  // WheelUnitから受信した状態バイト (bit0: mode≠0, bit1: 電源電圧範囲外, bit2: 過熱)
  uint8_t wheel_status[4];
  uint16_t wheel_frame_count[4];  // 正常に受信したフレーム数 (通信確認用、ラップアラウンドあり)
  // 受信を再開した回数 (app.c の HAL_UART_ErrorCallback と、OmniDrive_Recv の受信監視の合計)
  uint16_t wheel_rx_restart_count[4];
  uint32_t wheel_last_frame_tick[4];  // 最後に正常なフレームを受信した時刻 [ms] (受信監視用)
  bool wheel_rx_stalled[4];           // 受信が途絶えて再開を試みている最中か (診断ログを1回だけ出す用)
  bool rx_monitor_started;            // 受信監視の開始済みか (初回の OmniDrive_Recv で時刻を初期化する)
  // true: OmniDrive_SetVelEx を電圧制御 (フィードフォワードのみ、WheelUnitへ cmd 2) で出力する。
  // false (既定): 従来どおり WheelUnit の速度モード (cmd 1)。試合の経路は検証が済むまで false のまま
  bool use_voltage_control;
  float cmd_voltage[4];               // 直近に送った印加電圧 [V] (ログ用。速度モード中は0)
  // 電圧制御の機体速度PIの積分項 [V] (x, y: 並進、w: 回転。OmniDrive_SetFree で0に戻す)
  float vel_fb_integral[3];
  bool volt_saturated;                // 前周期にどれかの輪の電圧が上限に張り付いたか (積分を止める)
  MAF maf[4];
  // 順運動学行列: 逆運動学 H (行 [-sinθi, cosθi, R]) の最小二乗疑似逆行列 (HᵀH)⁻¹Hᵀ
  // 車輪線速度 [m/s] に掛けると [vx, vy, ω] が得られる
  float fk[3][4];
  TractionControl tcs;  // トラクションコントロールシステム
} OmniDrive;

void OmniDrive_Init(OmniDrive* self, Serial* serials);
void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle);
// vel_x, vel_y [mm/s], vel_angle [mrad/s]。imu が NULL の場合はS字加減速のみ (スリップ検知なし)
void OmniDrive_SetVelEx(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle,
                        const Imu* imu);
void OmniDrive_SetFree(OmniDrive* self);
// 電圧モードで各輪の印加電圧 [V] を送る (0Vは短絡ブレーキ)
void OmniDrive_SetVoltage(OmniDrive* self, const float volt[4]);
void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command);
void OmniDrive_Recv(OmniDrive* self);
// 実測車輪速度からの機体速度 vel_x, vel_y [mm/s], vel_angle [mrad/s]
void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle);
// 実測車輪速度からの機体速度 [m/s], [rad/s]
void OmniDrive_GetVelF(const OmniDrive* self, float* vx, float* vy, float* omega);

#endif  // __OMNI_DRIVE_H_
