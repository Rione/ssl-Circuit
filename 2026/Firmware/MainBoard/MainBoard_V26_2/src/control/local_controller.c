#include "local_controller.h"

#include <stdbool.h>
#include <stdio.h>

#include "mymath.h"

void LocalController_Init(LocalController* self) {
  (void)self;
}

void LocalController_Stop(LocalController* self, Robot* robot) {
  (void)self;
  OmniDrive_SetFree(&robot->omni_drive);
  // 信号ロスト/緊急停止時、受信済みの最後のキック指示が残っていると
  // ROBOT_KICK_INTERVAL_MSごとに再発火してしまうため、ここで明示的にクリアする。
  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendKicker(robot, &robot->info);  // キック指示をクリアするために送信

  Robot_SendDribble(robot, 0, 0);

  // 停止指示中でも慣性で一定速度以上動いている場合は、安全のためキック用コンデンサを放電する

  int16_t vel_x, vel_y, vel_angular;
  OmniDrive_GetVel(&robot->omni_drive, &vel_x, &vel_y, &vel_angular);
  int32_t speed_sq = (int32_t)vel_x * vel_x + (int32_t)vel_y * vel_y;
  if (speed_sq > (int32_t)ROBOT_STOP_DISCHARGE_SPEED_MMPS * ROBOT_STOP_DISCHARGE_SPEED_MMPS) {
    Kicker_Discharge(&robot->kicker);
  }
}

// 前(+x)→後(-x)→左(+y)→右(-y)の順に1秒ごとに切り替わる動作テスト
void LocalController_TestMove(LocalController* self, Robot* robot) {
  (void)self;
  static const int16_t kTestVel = 100;
  static const int16_t kDirVel[4][2] = {
      {kTestVel, 0},   // 前
      {0, kTestVel},   // 左
      {-kTestVel, 0},  // 後
      {0, -kTestVel},  // 右
  };
  static Timer timer = {0};
  static uint8_t phase = 0;

  if (Timer_ReadMs(&timer) >= 1000) {
    Timer_Reset(&timer);
    phase = (phase + 1) % 4;
  }
  OmniDrive_SetVel(&robot->omni_drive, kDirVel[phase][0], kDirVel[phase][1], 0);
}

// 0.3秒ごとに前(+x)/後(-x)を切り替える動作テスト
void LocalController_TestMoveForwardBack(LocalController* self, Robot* robot) {
  (void)self;
  static const int16_t kTestVel = 500;
  static Timer timer = {0};
  static uint8_t is_forward = 1;

  if (Timer_ReadMs(&timer) >= 300) {
    Timer_Reset(&timer);
    is_forward = !is_forward;
  }

  OmniDrive_SetVel(&robot->omni_drive, is_forward ? kTestVel : -kTestVel, 0, 0);
}

// TCS性能検証用テスト: 
// 1. 電源投入後10秒待機 (LED0が0.5s周期で点滅)
// 2. 10秒経過後テスト開始: 2000mm/sへの最速加減速で1.5m前後往復
//    - 1.5m反転ごとに TCS 有り(ON: LED0常時点灯) と TCS 無し(OFF: LED0高速点滅) を交互に切り替え
// 3. テスト開始から10秒経過 (起動後20秒以降) で完全自動停止 (LED0消灯)
void LocalController_TestTCSAcceleration(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static bool is_test_started = false;
  static bool is_tcs_enabled = true;   // true: TCS有り, false: TCS無し
  static int8_t direction = 1;         // +1: 前進(+x), -1: 後退(-x)
  static float x_pos_mm = 0.0f;        // スタート位置を原点とする絶対位置 [mm]
  static float initial_yaw_rad = 0.0f; // 走行開始時の基準姿勢角 [rad]

  const uint32_t kStartupWaitMs = 10000;   // 起動後待機時間 [ms] (10秒)
  const uint32_t kTestDurationMs = 10000;  // テスト走行時間 [ms] (10秒)
  const uint32_t kTotalTestTimeMs = kStartupWaitMs + kTestDurationMs; // 合計時間 [ms] (20秒)

  // 1. 安全確保: テスト中はキック・ドリブルを明示的にクリアし、放電状態を維持
  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  // 2. HAL_GetTick() による時間計測
  if (start_tick == 0) {
    start_tick = HAL_GetTick();
    if (start_tick == 0) start_tick = 1;
  }
  uint32_t elapsed_ms = HAL_GetTick() - start_tick;

  // 3. 起動後 0〜10秒 (kStartupWaitMs): 完全静止待機 (安全マージン)
  if (elapsed_ms < kStartupWaitMs) {
    OmniDrive_SetFree(&robot->omni_drive);

    // 待機中はLED0を0.5秒周期で点滅させ、カウントダウン中であることを明示
    if ((elapsed_ms / 500) % 2 == 0) {
      DigitalOut_Write(&robot->led0, 1);
    } else {
      DigitalOut_Write(&robot->led0, 0);
    }
    return;
  }

  // 4. テスト開始から10秒経過 (起動後20秒以降): 完全自動停止
  if (elapsed_ms >= kTotalTestTimeMs) {
    OmniDrive_SetFree(&robot->omni_drive);
    DigitalOut_Write(&robot->led0, 0);  // テスト完了により消灯
    return;
  }

  // 5. テスト初回開始時の初期化
  if (!is_test_started) {
    is_test_started = true;
    x_pos_mm = 0.0f;
    direction = 1;
    is_tcs_enabled = true;  // 最初はTCS有りでスタート
    initial_yaw_rad = robot->imu.yaw_rad;
    TCS_Reset(&robot->omni_drive.tcs);
  }

  // 6. 実測機体速度を取得し、スタート原点からの絶対位置を積算 (1ms周期)
  int16_t actual_vx_mmps, actual_vy_mmps, actual_omega;
  OmniDrive_GetVel(&robot->omni_drive, &actual_vx_mmps, &actual_vy_mmps, &actual_omega);

  const float dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;  // 0.001s
  x_pos_mm += (float)actual_vx_mmps * dt;

  // 7. 基準区間 (0 ~ 1500mm) 反転制御 ＆ TCS 有り/無しの交互切り替え
  const float kTargetDistance_mm = 1500.0f;
  if (direction == 1 && x_pos_mm >= kTargetDistance_mm) {
    direction = -1;                      // 前進限界到達 -> 後退へ反転
    is_tcs_enabled = !is_tcs_enabled;    // TCS 有り/無し を切り替え
    TCS_Reset(&robot->omni_drive.tcs);
  } else if (direction == -1 && x_pos_mm <= 0.0f) {
    direction = 1;                       // 原点帰還到達 -> 前進へ反転
    is_tcs_enabled = !is_tcs_enabled;    // TCS 有り/無し を切り替え
    TCS_Reset(&robot->omni_drive.tcs);
  }

  // 8. TCSの有効/無効フラグを動的に適用
  robot->omni_drive.tcs.config.enable_tcs = is_tcs_enabled;
  robot->omni_drive.tcs.config.enable_s_curve = is_tcs_enabled;

  // 9. LED0 表示: TCS有り時は常時点灯、TCS無し時は高速点滅(0.1s周期)で区別
  if (is_tcs_enabled) {
    DigitalOut_Write(&robot->led0, 1);
  } else {
    DigitalOut_Write(&robot->led0, (elapsed_ms / 100) % 2);
  }

  // 10. IMUヘディングロック (直進性の維持)
  float yaw_error = GapRadians(robot->imu.yaw_rad, initial_yaw_rad);
  float target_omega_rad = -2.5f * yaw_error;
  target_omega_rad = Constrain(target_omega_rad, -3.0f, 3.0f);
  int16_t target_omega = (int16_t)(target_omega_rad * 1000.0f);

  // 11. 2000mm/s 加速指令
  const int16_t kMaxSpeed_mmps = 2000;
  int16_t target_vx = kMaxSpeed_mmps * direction;

  OmniDrive_SetVelEx(&robot->omni_drive, target_vx, 0, target_omega,
                     robot->imu.yaw_rate, robot->info.battery_voltage);
}

