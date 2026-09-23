#include "local_controller.h"

#include <math.h>
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
//    - さらに60秒待って (ST-Linkを挿す時間) から記録をCSVでprintf出力 (USART1)
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
  static bool is_aborted = false;      // 安全停止した (以後は終了時と同じくブレーキして記録を出力)
  static uint32_t abort_elapsed_ms = 0;

  const uint32_t kStartupWaitMs = 10000;   // 起動後待機時間 [ms] (10秒)
  const uint32_t kTestDurationMs = 10000;  // テスト走行時間 [ms] (10秒)
  const uint32_t kTotalTestTimeMs = kStartupWaitMs + kTestDurationMs; // 合計時間 [ms] (20秒)
  const uint32_t kDumpDelayMs = 60000;     // テスト終了からCSV出力開始までの待機時間 [ms] (60秒)

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

  // 4. テスト開始から10秒経過 (起動後20秒以降) または安全停止: 完全自動停止
  if (elapsed_ms >= kTotalTestTimeMs || is_aborted) {
    OmniDrive_SetFree(&robot->omni_drive);

    uint32_t end_ms = is_aborted ? abort_elapsed_ms : kTotalTestTimeMs;
    uint32_t dump_wait_elapsed_ms = elapsed_ms - end_ms;
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
  // 反転 (減速開始) の位置は、今の速度から止まるのに要る距離 v²/(2×TEST_TCS_BRAKE_DECEL) だけ手前にする。
  // 区間の端で反転すると、3m/s では止まるまでに約1m行き過ぎた。行き過ぎを防ぐ向きに判定するため、
  // 前進はオドメトリと対地速度推定の位置の大きい方、後退は小さい方を使う。
  // 位置は区間の境界にスナップしない (手前で反転するので、スナップすると実際の位置からずれる)
  // 止まるのに要る距離は、その向きに進んでいるときだけ使う。以前は速度の大きさで計算していたため、
  // 後退へ切り替えた直後 (まだ前へ進んでいる) に後退側の判定も成り立って前進へ戻り、1msごとに
  // 前進と後退が入れ替わって減速が始まらず、行き過ぎていた
  float vx = robot->omni_drive.tcs.odom_vx;
  float stop_fwd_m = (vx > 0.0f) ? vx * vx / (2.0f * TEST_TCS_BRAKE_DECEL) : 0.0f;
  float stop_bwd_m = (vx < 0.0f) ? vx * vx / (2.0f * TEST_TCS_BRAKE_DECEL) : 0.0f;
  float fwd_progress_m = (x_pos_odom_m > x_pos_ground_m) ? x_pos_odom_m : x_pos_ground_m;
  float bwd_progress_m = (x_pos_odom_m < x_pos_ground_m) ? x_pos_odom_m : x_pos_ground_m;
  const float kTargetDistance_m = TEST_TCS_DISTANCE_M;
  if (direction == 1 && fwd_progress_m + stop_fwd_m >= kTargetDistance_m) {
    direction = -1;  // 前進側の端の手前 -> 後退へ反転
  } else if (direction == -1 && bwd_progress_m - stop_bwd_m <= 0.0f) {
    direction = 1;  // 原点の手前 -> 前進へ反転
  }

  // 8. TCSの介入 (accel_gain でS字の加速度上限を下げる、対地速度への指令の引き戻し) は速度モードのときだけ。
  //    電圧制御では輪ごとのトルク上限がトラクション制御を担うので、TCSは対地速度推定とスリップ判定だけ行う
  //    (介入を残すと、スリップ判定のたびに加速度上限が下がり、加速・減速が1.5m/s²程度しか出なかった)
  robot->omni_drive.tcs.config.enable_s_curve = true;
  // 出力を電圧制御にするか (parammeter.h の TEST_TCS_USE_VOLTAGE_CONTROL)。TCSの介入の有無と
  // S字の加速度上限もモードに合わせて切り替わる
  OmniDrive_SetControlMode(&robot->omni_drive, TEST_TCS_USE_VOLTAGE_CONTROL);

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

  // 安全停止: 向きが大きくずれた、または走行範囲を大きくはみ出したら、その場でブレーキして終了する
  // (電圧制御で機体が半回転して走り去ったことがあったため。記録は終了時と同じく後で出力する)
  if (fabsf(heading_yaw_rad) > TEST_TCS_ABORT_HEADING_RAD ||
      x_pos_odom_m > kTargetDistance_m + TEST_TCS_ABORT_OVERRUN_M ||
      x_pos_odom_m < -TEST_TCS_ABORT_OVERRUN_M) {
    is_aborted = true;
    abort_elapsed_ms = elapsed_ms;
    OmniDrive_SetFree(&robot->omni_drive);
    TcsLog_Record(elapsed_ms - kStartupWaitMs, true, 0, robot->imu.yaw_rate, &robot->omni_drive);
    printf("# TCS test aborted: t=%u heading=%d mrad x_odom=%d mm\n",
           (unsigned)(elapsed_ms - kStartupWaitMs), (int)(heading_yaw_rad * 1000.0f),
           (int)(x_pos_odom_m * 1000.0f));
    return;
  }

  // 11. 2000mm/s 加速指令
  const int16_t kMaxSpeed_mmps = TEST_TCS_SPEED_MMPS;
  int16_t target_vx = kMaxSpeed_mmps * direction;

  OmniDrive_SetVelEx(&robot->omni_drive, target_vx, 0, target_omega, &robot->imu);

  // 12. 10ms周期でTCS内部状態を記録
  if (++log_divider >= 10) {
    log_divider = 0;
    TcsLog_Record(elapsed_ms - kStartupWaitMs, true, target_vx, robot->imu.yaw_rate,
                  &robot->omni_drive);
  }
}

// 動作パターンのテスト (床で実施、電圧制御): 前後・左右・斜め・その場旋回・旋回しながらの並進を順に行う。
// スタート位置を原点、スタート時の前方を +x、左を +y とする床の座標で、各区間の移動量 (dx, dy) と
// 向きの変化 (dθ) を目標にして動く (位置は車輪オドメトリ、向きはジャイロの積分)。
// 目標への速度は min(上限, √(2×減速度×残り距離), P×残り距離) で決め、床の座標から機体の座標に直して
// OmniDrive_SetVelEx に渡す (旋回しながらでも床に対してまっすぐ進む)。
// 区間の終わり (残り3cm以内・向き3°以内) で0.3秒止まり、次の区間へ。
// 移動量は TEST_PATTERN_SCALE 倍する (1.0 で前後・左右1m)。
// 安全停止: 動く範囲 (x: -0.5〜倍率+0.5m, y: ±(倍率+0.5)m) を出た、向きのずれが大きい、区間が6秒で終わらない。
// 30ms周期で RAM に記録し (tcs_on 列 = 区間の番号)、終了60秒後に CSV を出力する (TCSテストと同じ形式)
typedef struct {
  float dx_m, dy_m;   // 床の座標での移動量 [m]
  float dtheta_rad;   // 向きの変化 [rad]
} MotionSegment;

static const MotionSegment kMotionSegments[] = {
    {+1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f},                        // 前後
    {0.0f, +1.0f, 0.0f},  {0.0f, -2.0f, 0.0f}, {0.0f, +1.0f, 0.0f},   // 左右
    {+0.7f, +0.7f, 0.0f}, {-0.7f, -0.7f, 0.0f},                       // 斜め (左前)
    {+0.7f, -0.7f, 0.0f}, {-0.7f, +0.7f, 0.0f},                       // 斜め (右前)
    {0.0f, 0.0f, +PI},    {0.0f, 0.0f, -PI},                          // その場旋回
    {+1.0f, 0.0f, +PI},   {-1.0f, 0.0f, -PI},                         // 旋回しながら前後
};

void LocalController_TestMotionPattern(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static bool is_started = false;
  static bool is_finished = false;
  static bool is_log_dumped = false;
  static uint32_t end_elapsed_ms = 0;
  static int seg = 0;
  static uint32_t seg_start_ms = 0;
  static uint32_t settled_ms = 0;          // 区間の終わりの条件を満たし始めた時刻 (0: 満たしていない)
  static float pos_x = 0.0f, pos_y = 0.0f, heading = 0.0f;  // 床の座標での位置 [m] と向き [rad]
  static float target_x = 0.0f, target_y = 0.0f, target_heading = 0.0f;
  static uint8_t log_divider = 0;
  static Timer dt_timer = {0};

  const uint32_t kStartupWaitMs = 10000;
  const uint32_t kDumpDelayMs = 60000;
  // 0.3秒 → 0.15秒。区間の切り替わりを詰める (速度そのものには関係ない)
  const uint32_t kSettleMs = 150;
  const uint32_t kSegmentTimeoutMs = 6000;
  // 位置・向きの目標への近づき方。P だけの範囲 (残り距離 < 2a/Kp²) が広いと最後がゆっくりになる
  // (Kp=3, a=2 では残り約0.44mから先で約0.9秒かかった)。
  // kBrakeAccel はトルク上限引き上げ (2.0→2.4→3.2→2.8V) に合わせて 3.0→3.5→5.0→4.5 にした。
  // 3.2V (kBrakeAccel=5.0) ではスリップが50〜87%まで増えたため、2.8Vに落ち着けた
  const float kBrakeAccel = 4.5f;      // 位置の目標へ減速するときの想定減速度 [m/s^2]
  const float kPosKp = 8.0f;           // [1/s]
  const float kMaxAngVel = TEST_PATTERN_ANG_VEL_RADPS;   // [rad/s]
  const float kBrakeAngAccel = TEST_PATTERN_ANG_BRAKE;  // [rad/s^2]
  const float kHeadingKp = 10.0f;      // [1/s]
  const float kHeadingKd = 0.5f;       // 角速度による減衰 [s] (オーバーシュート対策)
  const int kNumSegments = sizeof(kMotionSegments) / sizeof(kMotionSegments[0]);

  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  if (start_tick == 0) {
    start_tick = HAL_GetTick();
    if (start_tick == 0) start_tick = 1;
  }
  uint32_t elapsed_ms = HAL_GetTick() - start_tick;
  OmniDrive* od = &robot->omni_drive;

  if (elapsed_ms < kStartupWaitMs) {
    OmniDrive_SetFree(od);
    DigitalOut_Write(&robot->led0, (elapsed_ms / 500) % 2 == 0);
    return;
  }

  // 終了 (全区間を終えた、または安全停止) 後はブレーキし、60秒後に記録を出力する
  if (is_finished) {
    OmniDrive_SetFree(od);
    uint32_t wait_ms = elapsed_ms - end_elapsed_ms;
    if (wait_ms < kDumpDelayMs) {
      DigitalOut_Write(&robot->led0, (wait_ms / 500) % 2 == 0);
      return;
    }
    DigitalOut_Write(&robot->led0, 0);
    if (!is_log_dumped) is_log_dumped = TcsLog_DumpStep();
    return;
  }

  if (!is_started) {
    is_started = true;
    TCS_Reset(&od->tcs);
    TcsLog_Reset();
    Timer_Init(&dt_timer);
    Timer_Reset(&dt_timer);
    seg = 0;
    seg_start_ms = elapsed_ms;
    target_x = kMotionSegments[0].dx_m * TEST_PATTERN_SCALE;
    target_y = kMotionSegments[0].dy_m * TEST_PATTERN_SCALE;
    target_heading = kMotionSegments[0].dtheta_rad;
  }

  float dt = Timer_Read(&dt_timer);
  Timer_Reset(&dt_timer);
  if (dt <= 0.0f || dt > 0.05f) dt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;

  // 位置と向きの推定 (機体座標のオドメトリ速度を床の座標に直して積分)
  heading += robot->imu.yaw_rate * dt;
  float c = cosf(heading), s = sinf(heading);
  float vx_body = od->tcs.odom_vx, vy_body = od->tcs.odom_vy;
  pos_x += (vx_body * c - vy_body * s) * dt;
  pos_y += (vx_body * s + vy_body * c) * dt;

  // 目標への速度指令 (床の座標)
  float ex = target_x - pos_x, ey = target_y - pos_y;
  float dist = sqrtf(ex * ex + ey * ey);
  float speed = fminf(TEST_PATTERN_SPEED_MPS, fminf(sqrtf(2.0f * kBrakeAccel * dist), kPosKp * dist));
  float vx_world = (dist > 1e-3f) ? ex / dist * speed : 0.0f;
  float vy_world = (dist > 1e-3f) ? ey / dist * speed : 0.0f;
  // 向きの目標への近づき方は P (残り角度に比例) だけでなく、実際の角速度 (ジャイロ) で減衰させる
  // D項も入れる (LocalController_TestTCSAcceleration のヘディング保持と同じ形)。P だけだと、
  // 「この速さなら間に合う」という想定 (kBrakeAngAccel) と実際に減速できる速さがずれたときに
  // 行きすぎては戻る振動になる (3.2Vのテストで±37°のリンギングが出た)。D項は誤差の大小に関わらず
  // 常に効くので、想定と実際がずれていても行きすぎを抑えられる
  float eh = target_heading - heading;
  float ang_speed =
      fminf(kMaxAngVel, fminf(sqrtf(2.0f * kBrakeAngAccel * fabsf(eh)), kHeadingKp * fabsf(eh)));
  float omega = (eh >= 0.0f) ? ang_speed : -ang_speed;
  omega -= kHeadingKd * robot->imu.yaw_rate;
  omega = Constrain(omega, -kMaxAngVel, kMaxAngVel);
  // 床の座標から機体の座標へ
  float vx_cmd = vx_world * c + vy_world * s;
  float vy_cmd = -vx_world * s + vy_world * c;

  // 安全停止
  const MotionSegment* ms = &kMotionSegments[seg];
  const float kReach = TEST_PATTERN_SCALE;  // 区間の最大の移動量 (前・左右) [m]
  bool out_of_area =
      pos_x < -0.5f || pos_x > kReach + 0.5f || pos_y < -(kReach + 0.5f) || pos_y > kReach + 0.5f;
  bool heading_off = fabsf(eh) > fabsf(ms->dtheta_rad) + 0.8f;
  bool timeout = (elapsed_ms - seg_start_ms) > kSegmentTimeoutMs;
  if (out_of_area || heading_off || timeout) {
    OmniDrive_SetFree(od);
    TcsLog_Record(elapsed_ms - kStartupWaitMs, (uint8_t)(seg + 1), 0, robot->imu.yaw_rate, od);
    printf("# motion test aborted: seg=%d area=%d heading=%d timeout=%d x=%d y=%d mm th=%d mrad\n",
           seg + 1, out_of_area, heading_off, timeout, (int)(pos_x * 1000.0f),
           (int)(pos_y * 1000.0f), (int)(heading * 1000.0f));
    is_finished = true;
    end_elapsed_ms = elapsed_ms;
    return;
  }

  OmniDrive_SetControlMode(od, true);
  od->tcs.config.enable_s_curve = true;
  OmniDrive_SetVelEx(od, (int16_t)(vx_cmd * 1000.0f), (int16_t)(vy_cmd * 1000.0f),
                     (int16_t)(omega * 1000.0f), &robot->imu);
  DigitalOut_Write(&robot->led0, 1);

  if (++log_divider >= 30) {  // 30ms周期 (1000サンプルで30秒分。全区間で約25秒かかる)
    log_divider = 0;
    TcsLog_Record(elapsed_ms - kStartupWaitMs, (uint8_t)(seg + 1), (int16_t)(vx_cmd * 1000.0f),
                  robot->imu.yaw_rate, od);
  }

  // 区間の終わり: 残り3cm以内・向き3°以内で0.3秒保ったら次の区間へ
  if (dist < 0.03f && fabsf(eh) < 0.05f) {
    if (settled_ms == 0) settled_ms = elapsed_ms;
    if (elapsed_ms - settled_ms >= kSettleMs) {
      settled_ms = 0;
      seg++;
      if (seg >= kNumSegments) {
        is_finished = true;
        end_elapsed_ms = elapsed_ms;
        printf("# motion test finished\n");
        return;
      }
      seg_start_ms = elapsed_ms;
      target_x += kMotionSegments[seg].dx_m * TEST_PATTERN_SCALE;
      target_y += kMotionSegments[seg].dy_m * TEST_PATTERN_SCALE;
      target_heading += kMotionSegments[seg].dtheta_rad;
    }
  } else {
    settled_ms = 0;
  }
}

// 電圧モードの単体評価用テスト (機体を浮かせて実施、4輪に同じ電圧をかける):
// 起動5秒後から下の区間表を1回だけ流し、10ms周期で指令電圧と実測車輪速度をCSVでprintf出力する (USART1)。
//   1. 定常の階段 (±0.5〜4V) … 逆起電力定数・摩擦電圧・前進後退の対称性
//   2. ステップ (0→+2V→0→-2V→0) … 応答の時定数、0V (短絡ブレーキ) での減速
//   3. ゆっくりしたランプ (0→±1.2V を6秒) … 動き出す電圧 (デッドバンド)
// 最大4Vに抑える (WheelUnitのホイールロック検知は +5.0V ちょうどが1秒続くと出力を切る)
typedef struct {
  uint16_t duration_ms;
  float v_start;  // 区間開始時の電圧 [V]
  float v_end;    // 区間終了時の電圧 [V] (v_start と同じなら一定、違えば直線で変化)
} VoltageTestSegment;

static const VoltageTestSegment kVoltageTestSegments[] = {
    // 1. 定常の階段 (正→負)
    {1500, 0.5f, 0.5f}, {1500, 1.0f, 1.0f}, {1500, 1.5f, 1.5f},
    {1500, 2.0f, 2.0f}, {1500, 3.0f, 3.0f}, {1500, 4.0f, 4.0f},
    {1000, 0.0f, 0.0f},
    {1500, -0.5f, -0.5f}, {1500, -1.0f, -1.0f}, {1500, -1.5f, -1.5f},
    {1500, -2.0f, -2.0f}, {1500, -3.0f, -3.0f}, {1500, -4.0f, -4.0f},
    {1000, 0.0f, 0.0f},
    // 2. ステップ
    {1500, 2.0f, 2.0f}, {1500, 0.0f, 0.0f}, {1500, -2.0f, -2.0f}, {1500, 0.0f, 0.0f},
    // 3. ゆっくりしたランプ
    {6000, 0.0f, 1.2f}, {1000, 0.0f, 0.0f}, {6000, 0.0f, -1.2f}, {1000, 0.0f, 0.0f},
};

void LocalController_TestVoltage(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static uint32_t last_log_ms = 0;
  static bool is_header_printed = false;
  static bool is_end_printed = false;
  static uint16_t rx_restart_base = 0;

  const uint32_t kStartupWaitMs = 5000;
  const uint32_t kLogIntervalMs = 10;
  const int kNumSegments = sizeof(kVoltageTestSegments) / sizeof(kVoltageTestSegments[0]);

  // 安全確保: テスト中はキック・ドリブルを明示的にクリアし、放電状態を維持
  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  if (start_tick == 0) {
    start_tick = HAL_GetTick();
    if (start_tick == 0) start_tick = 1;
  }
  uint32_t elapsed_ms = HAL_GetTick() - start_tick;
  OmniDrive* od = &robot->omni_drive;

  if (elapsed_ms < kStartupWaitMs) {
    OmniDrive_SetFree(od);
    DigitalOut_Write(&robot->led0, (elapsed_ms / 500) % 2 == 0);
    return;
  }

  // 現在の区間と指令電圧を求める
  uint32_t t_ms = elapsed_ms - kStartupWaitMs;
  uint32_t seg_start_ms = 0;
  int seg = 0;
  while (seg < kNumSegments && t_ms >= seg_start_ms + kVoltageTestSegments[seg].duration_ms) {
    seg_start_ms += kVoltageTestSegments[seg].duration_ms;
    seg++;
  }
  if (seg >= kNumSegments) {
    OmniDrive_SetFree(od);
    DigitalOut_Write(&robot->led0, 0);
    if (!is_end_printed) {
      printf("# voltage test end\n");
      is_end_printed = true;
    }
    return;
  }
  const VoltageTestSegment* s = &kVoltageTestSegments[seg];
  float ratio = (float)(t_ms - seg_start_ms) / (float)s->duration_ms;
  float volt = s->v_start + (s->v_end - s->v_start) * ratio;
  const float volts[4] = {volt, volt, volt, volt};
  OmniDrive_SetVoltage(od, volts);
  DigitalOut_Write(&robot->led0, 1);

  // 10ms周期でログ出力 (s: 4輪の状態バイトを1桁ずつ並べた16進、vbat: 電源電圧 [V]、
  // rx: テスト開始からの受信再開回数 (4輪合計))
  uint16_t rx_restart = 0;
  for (int i = 0; i < 4; i++) rx_restart += od->wheel_rx_restart_count[i];
  if (!is_header_printed) {
    printf("t_ms,seg,v_x100,w0_x100,w1_x100,w2_x100,w3_x100,s,vbat,rx\n");
    is_header_printed = true;
    rx_restart_base = rx_restart;
    last_log_ms = t_ms;
  } else if (t_ms - last_log_ms < kLogIntervalMs) {
    return;
  }
  last_log_ms = t_ms;
  unsigned status = (od->wheel_status[0] & 0xFU) | ((od->wheel_status[1] & 0xFU) << 4) |
                    ((od->wheel_status[2] & 0xFU) << 8) | ((od->wheel_status[3] & 0xFU) << 12);
  printf("%u,%d,%d,%d,%d,%d,%d,%04X,%u,%u\n", (unsigned)t_ms, seg, (int)(volt * 100.0f),
         (int)(od->vel_wheel_angular[0] * 100.0f), (int)(od->vel_wheel_angular[1] * 100.0f),
         (int)(od->vel_wheel_angular[2] * 100.0f), (int)(od->vel_wheel_angular[3] * 100.0f),
         status, (unsigned)robot->info.battery_voltage,
         (unsigned)(uint16_t)(rx_restart - rx_restart_base));
}

// 電圧制御 (フィードフォワードのみ) を浮かせて確かめるテスト:
// 起動5秒後から、機体速度で前後・左右・旋回を順に指令し (OmniDrive_SetVelEx を電圧制御で出力)、
// 10ms周期で目標車輪角速度・実測・印加電圧をCSVでprintf出力する (USART1)。
// 無負荷なので、Ke・Vf の表と逆運動学が正しければ実測は目標にほぼ一致する
typedef struct {
  uint16_t duration_ms;
  int16_t vx_mmps, vy_mmps, omega_mradps;
} VelocityTestSegment;

static const VelocityTestSegment kVoltageFFTestSegments[] = {
    {3000, 2000, 0, 0},   {1000, 0, 0, 0}, {3000, -2000, 0, 0}, {1000, 0, 0, 0},
    {3000, 0, 1500, 0},   {1000, 0, 0, 0}, {3000, 0, -1500, 0}, {1000, 0, 0, 0},
    {3000, 0, 0, 10000},  {1000, 0, 0, 0}, {3000, 0, 0, -10000}, {1000, 0, 0, 0},
};

void LocalController_TestVoltageFF(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static uint32_t last_log_ms = 0;
  static bool is_header_printed = false;
  static bool is_end_printed = false;

  const uint32_t kStartupWaitMs = 5000;
  const uint32_t kLogIntervalMs = 10;
  const int kNumSegments = sizeof(kVoltageFFTestSegments) / sizeof(kVoltageFFTestSegments[0]);

  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  if (start_tick == 0) {
    start_tick = HAL_GetTick();
    if (start_tick == 0) start_tick = 1;
  }
  uint32_t elapsed_ms = HAL_GetTick() - start_tick;
  OmniDrive* od = &robot->omni_drive;

  if (elapsed_ms < kStartupWaitMs) {
    OmniDrive_SetFree(od);
    DigitalOut_Write(&robot->led0, (elapsed_ms / 500) % 2 == 0);
    return;
  }

  uint32_t t_ms = elapsed_ms - kStartupWaitMs;
  uint32_t seg_start_ms = 0;
  int seg = 0;
  while (seg < kNumSegments && t_ms >= seg_start_ms + kVoltageFFTestSegments[seg].duration_ms) {
    seg_start_ms += kVoltageFFTestSegments[seg].duration_ms;
    seg++;
  }
  if (seg >= kNumSegments) {
    od->use_voltage_control = false;
    OmniDrive_SetFree(od);
    DigitalOut_Write(&robot->led0, 0);
    if (!is_end_printed) {
      printf("# voltage FF test end\n");
      is_end_printed = true;
    }
    return;
  }

  // IMU は渡さない (浮かせているので対地速度推定・スリップ検知は使わず、S字加減速だけ)
  const VelocityTestSegment* s = &kVoltageFFTestSegments[seg];
  od->use_voltage_control = true;
  OmniDrive_SetVelEx(od, s->vx_mmps, s->vy_mmps, s->omega_mradps, NULL);
  DigitalOut_Write(&robot->led0, 1);

  if (!is_header_printed) {
    fputs("t_ms,seg,t0_x100,t1_x100,t2_x100,t3_x100,w0_x100,w1_x100,w2_x100,w3_x100,", stdout);
    fflush(stdout);
    fputs("v0_x100,v1_x100,v2_x100,v3_x100,s,rx\n", stdout);
    is_header_printed = true;
    last_log_ms = t_ms;
  } else if (t_ms - last_log_ms < kLogIntervalMs) {
    return;
  }
  last_log_ms = t_ms;
  // s: 4輪の状態バイトを1桁ずつ並べた16進 (bit0: mode≠0, bit1: 電源電圧範囲外, bit2: 過熱)
  // rx: 受信再開回数 (4輪合計、起動から)
  unsigned status = (od->wheel_status[0] & 0xFU) | ((od->wheel_status[1] & 0xFU) << 4) |
                    ((od->wheel_status[2] & 0xFU) << 8) | ((od->wheel_status[3] & 0xFU) << 12);
  unsigned rx_restart = 0;
  for (int i = 0; i < 4; i++) rx_restart += od->wheel_rx_restart_count[i];
  printf("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%04X,%u\n", (unsigned)t_ms, seg,
         (int)(od->target_wheel_angular[0] * 100.0f), (int)(od->target_wheel_angular[1] * 100.0f),
         (int)(od->target_wheel_angular[2] * 100.0f), (int)(od->target_wheel_angular[3] * 100.0f),
         (int)(od->vel_wheel_angular[0] * 100.0f), (int)(od->vel_wheel_angular[1] * 100.0f),
         (int)(od->vel_wheel_angular[2] * 100.0f), (int)(od->vel_wheel_angular[3] * 100.0f),
         (int)(od->cmd_voltage[0] * 100.0f), (int)(od->cmd_voltage[1] * 100.0f),
         (int)(od->cmd_voltage[2] * 100.0f), (int)(od->cmd_voltage[3] * 100.0f), status,
         rx_restart);
}

// WheelUnitのID・回転方向・受信チャンネル確認用テスト (機体を浮かせて実施):
// 1. 電源投入後5秒待機 (LED0が0.5s周期で点滅)
// 2. ID1→ID4の順に1輪ずつ、+8rad/s (2秒) → -8rad/s (2秒) → 停止 (1秒) を繰り返す (回転中はLED0点灯)
//    - 逆運動学 v = -vx·sinθ + vy·cosθ + R·ω より、+指令は上から見て機体をCCWに回す向き
//      (ID1=左前55°, ID2=左後135°, ID3=右後-135°, ID4=右前-55°)
//    - TCS・S字は通さず、車輪角速度指令を直接送る
// 3. 100ms周期で指令・実測車輪速度・状態バイト・受信フレーム数・IMUをCSVでprintf出力 (USART1)
//    - f0〜f3 は直近100msの受信フレーム数 (WheelUnitの送信周期500µsなので約200が正常)
void LocalController_TestWheelSpin(LocalController* self, Robot* robot) {
  (void)self;
  static uint32_t start_tick = 0;
  static bool is_header_printed = false;
  static uint32_t last_log_ms = 0;
  static uint16_t prev_frame_count[4] = {0};

  const uint32_t kStartupWaitMs = 5000;   // 起動後待機時間 [ms]
  const uint32_t kSpinMs = 2000;          // 1方向あたりの回転時間 [ms]
  const uint32_t kStopMs = 1000;          // 輪ごとの停止時間 [ms]
  const uint32_t kWheelPeriodMs = kSpinMs * 2 + kStopMs;
  const int16_t kSpinSpeed_x100 = 800;    // 8rad/s (WheelUnitへの指令は ×0.01 rad/s)
  const uint32_t kLogIntervalMs = 100;    // ログ出力周期 [ms]

  // 1. 安全確保: テスト中はキック・ドリブルを明示的にクリアし、放電状態を維持
  robot->info.kicker.straight = 0;
  robot->info.kicker.chip = 0;
  robot->info.status.do_direct_straight = 0;
  robot->info.status.do_direct_chip = 0;
  Robot_SendDribble(robot, 0, 0);
  Kicker_Discharge(&robot->kicker);

  if (start_tick == 0) {
    start_tick = HAL_GetTick();
    if (start_tick == 0) start_tick = 1;
  }
  uint32_t elapsed_ms = HAL_GetTick() - start_tick;

  // 2. 現在の区間から、回す輪と指令を決める
  int8_t wheel = -1;  // 回す輪のインデックス (-1: 全輪停止)
  int16_t cmd = 0;
  if (elapsed_ms >= kStartupWaitMs) {
    uint32_t cycle_ms = (elapsed_ms - kStartupWaitMs) % (kWheelPeriodMs * 4);
    uint32_t phase_ms = cycle_ms % kWheelPeriodMs;
    if (phase_ms < kSpinMs) {
      cmd = kSpinSpeed_x100;
    } else if (phase_ms < kSpinMs * 2) {
      cmd = -kSpinSpeed_x100;
    }
    if (cmd != 0) wheel = (int8_t)(cycle_ms / kWheelPeriodMs);
  }

  if (wheel >= 0) {
    int16_t m[4] = {0, 0, 0, 0};
    m[wheel] = cmd;
    OmniDrive_Send(&robot->omni_drive, m, 1);  // command: 1 (速度モード)
    DigitalOut_Write(&robot->led0, 1);
  } else {
    OmniDrive_SetFree(&robot->omni_drive);
    if (elapsed_ms < kStartupWaitMs && (elapsed_ms / 500) % 2 == 0) {
      DigitalOut_Write(&robot->led0, 1);
    } else {
      DigitalOut_Write(&robot->led0, 0);
    }
  }

  // 3. 100ms周期でログ出力
  const OmniDrive* od = &robot->omni_drive;
  if (!is_header_printed) {
    fputs("t_ms,wheel_id,cmd_x100,w0_x100,w1_x100,w2_x100,w3_x100,s0,s1,s2,s3,f0,f1,f2,f3,",
          stdout);
    fflush(stdout);
    fputs("e0,e1,e2,e3,gyro_mrad,yaw_mrad,ax_cm,ay_cm\n", stdout);
    is_header_printed = true;
    for (int i = 0; i < 4; i++) prev_frame_count[i] = od->wheel_frame_count[i];
    last_log_ms = elapsed_ms;
    return;
  }
  if (elapsed_ms - last_log_ms < kLogIntervalMs) return;
  last_log_ms = elapsed_ms;

  uint16_t frames[4];
  for (int i = 0; i < 4; i++) {
    frames[i] = (uint16_t)(od->wheel_frame_count[i] - prev_frame_count[i]);
    prev_frame_count[i] = od->wheel_frame_count[i];
  }
  // IMU列 (静止中のバイアス残差・ドリフト確認用): 角速度 [mrad/s]、Madgwickヨー角 [mrad]、
  // 機体座標のバイアス補正済み加速度 [cm/s^2]
  const Imu* imu = &robot->imu;
  printf("%u,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d\n",
         (unsigned)elapsed_ms, wheel + 1, cmd, (int)(od->vel_wheel_angular[0] * 100.0f),
         (int)(od->vel_wheel_angular[1] * 100.0f), (int)(od->vel_wheel_angular[2] * 100.0f),
         (int)(od->vel_wheel_angular[3] * 100.0f), od->wheel_status[0], od->wheel_status[1],
         od->wheel_status[2], od->wheel_status[3], frames[0], frames[1], frames[2], frames[3],
         od->wheel_rx_restart_count[0], od->wheel_rx_restart_count[1],
         od->wheel_rx_restart_count[2], od->wheel_rx_restart_count[3],
         (int)(imu->yaw_rate * 1000.0f), (int)(imu->yaw_rad * 1000.0f),
         (int)(imu->accel_robot_x * 100.0f), (int)(imu->accel_robot_y * 100.0f));
}

