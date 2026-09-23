#include "local_controller.h"

#include <stdbool.h>
#include <stdio.h>

#include "mymath.h"
#include "tcs_log.h"

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
// 2. 10秒経過後テスト開始: 2000mm/sへの最速加減速で1.5m前後往復 (前後ともTCS常時有効)
//    - 10ms周期でTCS内部状態をRAMに記録
// 3. テスト開始から10秒経過 (起動後20秒以降) で完全自動停止 (LED0消灯)
//    - 停止中に記録をCSVでprintf出力 (USART1)
void LocalController_TestTCSAcceleration(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static bool is_test_started = false;
  static int8_t direction = 1;         // +1: 前進(+x), -1: 後退(-x)
  static float x_pos_odom_m = 0.0f;    // 車輪オドメトリ由来の絶対位置 [m] (スタート位置=0)
  static float x_pos_ground_m = 0.0f;  // IMU融合の対地速度由来の絶対位置 [m] (スタート位置=0)
  static float heading_yaw_rad = 0.0f; // 走行開始を基準0とする純ジャイロ積分ヘディング [rad]
  static uint8_t log_divider = 0;
  static bool is_log_dumped = false;

  const uint32_t kStartupWaitMs = 10000;   // 起動後待機時間 [ms] (10秒)
  const uint32_t kTestDurationMs = 10000;  // テスト走行時間 [ms] (10秒)
  const uint32_t kTotalTestTimeMs = kStartupWaitMs + kTestDurationMs; // 合計時間 [ms] (20秒)
  const uint32_t kDumpDelayMs = 40000;     // テスト終了からCSV出力開始までの待機時間 [ms] (40秒)

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

    uint32_t dump_wait_elapsed_ms = elapsed_ms - kTotalTestTimeMs;
    if (dump_wait_elapsed_ms < kDumpDelayMs) {
      // CSV出力開始までの待機中: シリアルターミナルの準備時間として0.5秒周期で点滅
      if ((dump_wait_elapsed_ms / 500) % 2 == 0) {
        DigitalOut_Write(&robot->led0, 1);
      } else {
        DigitalOut_Write(&robot->led0, 0);
      }
      return;
    }

    // 停止後に記録を1ループ1行ずつ出力 (1行≈3ms。SetFreeは毎ループ送り続ける)
    DigitalOut_Write(&robot->led0, 0);  // 出力中は消灯
    if (!is_log_dumped) {
      is_log_dumped = TcsLog_DumpStep();
    }
    return;
  }

  // 5. テスト初回開始時の初期化
  if (!is_test_started) {
    is_test_started = true;
    x_pos_odom_m = 0.0f;
    x_pos_ground_m = 0.0f;
    direction = 1;
    heading_yaw_rad = 0.0f;
    TCS_Reset(&robot->omni_drive.tcs);
    TcsLog_Reset();
  }

  // 6. スタート原点からの絶対位置を積算
  // 反転判定には車輪オドメトリ(odom_vx)を使う。IMU融合の対地速度(ground_vx)は
  // is_slipping中は車輪オドメトリの補正を受けずIMU積分のみになる仕様のため、
  // TCS OFF区間(S字無効=急反転で容易にis_slipping=trueへ張り付く)でIMU較正誤差や
  // 加速度センサ飽和の影響を受けやすく、反転条件に到達できず後退し続ける不具合があった。
  // 空転時に距離をやや過大に数える誤差はあるが、反転トリガーとしては十分な精度。
  // dtは実測 (制御ループは基本1ms周期だが、Rock5A SPIストール検知等のブロッキングで
  // 稀に周期が乱れる。固定値だと乱れた分だけ位置・ヘディング積分が誤るため)
  static Timer dt_timer = {0};
  static bool dt_timer_initialized = false;
  float dt;
  if (!dt_timer_initialized) {
    Timer_Init(&dt_timer);
    Timer_Reset(&dt_timer);
    dt_timer_initialized = true;
    dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;
  } else {
    dt = Timer_Read(&dt_timer);
    Timer_Reset(&dt_timer);
    if (dt <= 0.0f) dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;
    if (dt > 0.05f) dt = 0.05f;  // 50ms超のギャップはクランプ (異常時の暴走防止)
  }
  x_pos_odom_m += robot->omni_drive.tcs.odom_vx * dt;
  x_pos_ground_m += robot->omni_drive.tcs.ground_vx * dt;

  // 7. 基準区間 (0 ~ 1.5m) 反転制御
  // ※ ここで TCS_Reset すると平滑化状態が0に戻り、走行中に0指令へステップしてしまうため呼ばない
  //
  // odom_vxとground_vx(IMU融合の対地速度)の両方が到達を示すまで待つことで、
  // 急反転時の車輪スリップにより片方だけが早期に「到達した」と示しても反転しない
  // ようにする(多少反転が遅れる方向にのみ誤差が出る、行き過ぎ防止)
  float fwd_progress_m = (x_pos_odom_m < x_pos_ground_m) ? x_pos_odom_m : x_pos_ground_m;
  float bwd_progress_m = (x_pos_odom_m > x_pos_ground_m) ? x_pos_odom_m : x_pos_ground_m;
  const float kTargetDistance_m = 1.5f;
  if (direction == 1 && fwd_progress_m >= kTargetDistance_m) {
    direction = -1;  // 前進限界到達 -> 後退へ反転
    // 誤差の蓄積を防ぐため、区間の境界値に位置をスナップしてから次区間を開始する
    x_pos_odom_m = kTargetDistance_m;
    x_pos_ground_m = kTargetDistance_m;
  } else if (direction == -1 && bwd_progress_m <= 0.0f) {
    direction = 1;  // 原点帰還到達 -> 前進へ反転
    x_pos_odom_m = 0.0f;
    x_pos_ground_m = 0.0f;
  }

  // 8. TCSは前後どちらの区間でも常時有効 (ON/OFF比較はやめ、TCS自体の改善に集中する)
  robot->omni_drive.tcs.config.enable_tcs = true;
  robot->omni_drive.tcs.config.enable_s_curve = true;

  // 9. LED0 表示: テスト走行中は常時点灯
  DigitalOut_Write(&robot->led0, 1);

  // 10. IMUヘディングロック (直進性の維持)
  // imu.yaw_rad (Madgwick, 加速度計で重力方向を補正) は、S字加減速や急反転による
  // 大きな並進加速度がかかると重力方向の推定ごと乱れ、その誤差がヨー角にも漏れて
  // 実際には曲がっていないのに操舵してしまい、経路が左右に振れる原因になっていた。
  // そのためヘディング基準は加速度計の影響を受けない純ジャイロ積分(ヨーレートの
  // 時間積分)で保持する (10秒程度の短時間試験ではジャイロドリフトは無視できる)。
  // また比例制御のみだと遅れにより振動しやすいため、角速度フィードバック(D項)で
  // オーバーシュートを抑える (PD制御)
  heading_yaw_rad += robot->imu.yaw_rate * dt;
  const float kHeadingKp = 2.5f;
  const float kHeadingKd = 0.2f;
  float target_omega_rad = -kHeadingKp * heading_yaw_rad - kHeadingKd * robot->imu.yaw_rate;
  target_omega_rad = Constrain(target_omega_rad, -3.0f, 3.0f);
  int16_t target_omega = (int16_t)(target_omega_rad * 1000.0f);

  // 11. 2000mm/s 加速指令
  const int16_t kMaxSpeed_mmps = 2000;
  int16_t target_vx = kMaxSpeed_mmps * direction;

  OmniDrive_SetVelEx(&robot->omni_drive, target_vx, 0, target_omega, &robot->imu);

  // 12. 10ms周期でTCS内部状態を記録
  if (++log_divider >= 10) {
    log_divider = 0;
    TcsLog_Record(elapsed_ms - kStartupWaitMs, true, target_vx, robot->imu.yaw_rate,
                  &robot->omni_drive);
  }
}

