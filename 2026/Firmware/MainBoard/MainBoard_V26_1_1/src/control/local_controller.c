#include "local_controller.h"

#include <stdio.h>

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

// ===========================================================================
// ドリブラ保持力・干渉チェック試験
// ===========================================================================
// ドリブラ基板がボールを保持している間、後退速度を 0 から徐々に上げていき、
// どの速度で保持が破れるかを見る。外れたらゆっくり前進して拾い直し、
// 再捕捉したら後退ランプを 0 からやり直す。
//
// ドリブラ側は FW/Dribbler_V26_1_1-holdtest を書き込んでおくこと。
// あちらが自前でフォトセンサを見て滑らかに立ち上げ、保持状態を
// CAN(ID 0x70) の bit0 で送ってくる。
//
// 安全のため、電源投入直後は動かない。最初にボールを検知して初めて動き出す。

// 後退の最大速度[%] (ROBOT_MAX_VEL に対する割合)
// ROBOT_MAX_VEL = 3.0m/s なので 25% = 750mm/s
#define HOLDTEST_MAX_RETREAT_PERCENT 25
// 0 から最大速度まで上げる時間[ms]
#define HOLDTEST_RETREAT_RAMP_MS 2000
// 1サイクルあたりの後退時間の上限[ms]。走りすぎ防止
#define HOLDTEST_MAX_RETREAT_MS 2500
// ボールを拾いに行くときの前進速度[mm/s]
#define HOLDTEST_APPROACH_MMPS 150
// 前進し続ける時間の上限[ms]。これを超えたら一旦停止する
#define HOLDTEST_MAX_APPROACH_MS 3000

void LocalController_TestBallHold(LocalController* self, Robot* robot) {
  (void)self;

  // ROBOT_MAX_VEL[m/s] の割合を mm/s に直す
  static const int16_t kMaxRetreatMmps =
      (int16_t)(ROBOT_MAX_VEL * 1000.0f * HOLDTEST_MAX_RETREAT_PERCENT / 100.0f);

  enum { ST_IDLE, ST_APPROACH, ST_RETREAT };
  static uint8_t state = ST_IDLE;
  static Timer timer = {0};
  static uint8_t armed = 0;
  static uint16_t cycle = 0;

  uint8_t ball = robot->info.dribble_status.is_detected_ball;

  // 緊急停止中は何もしない。解除されても再度ボールを置くまで動かない
  if (robot->info.status.emergency_stop) {
    OmniDrive_SetFree(&robot->omni_drive);
    Robot_SendDribble(robot, 0, 0);
    state = ST_IDLE;
    armed = 0;
    return;
  }

  // ドリブラを回す指示は出し続ける
  // (holdtest FW のドリブラはCAN指令を無視して自走するが、
  //  通常FWのドリブラでも動くようにここでも送っておく)
  Robot_SendDribble(robot, 1, 0);

  // 電源投入直後は停止。最初にボールを検知したら試験開始
  if (!armed) {
    OmniDrive_SetVel(&robot->omni_drive, 0, 0, 0);
    if (ball) {
      armed = 1;
      state = ST_RETREAT;
      cycle = 1;
      Timer_Reset(&timer);
      printf("[HOLDTEST] 開始 サイクル%u 後退ランプ 0 -> %dmm/s\r\n", cycle,
             kMaxRetreatMmps);
    }
    return;
  }

  switch (state) {
    case ST_RETREAT: {
      uint32_t elapsed = Timer_ReadMs(&timer);

      if (!ball) {
        // 保持が破れた。そのときの速度を記録して前進へ
        int16_t vel = (elapsed >= HOLDTEST_RETREAT_RAMP_MS)
                          ? kMaxRetreatMmps
                          : (int16_t)((int32_t)kMaxRetreatMmps * elapsed /
                                      HOLDTEST_RETREAT_RAMP_MS);
        printf("[HOLDTEST] サイクル%u 保持が破れた: 後退%dmm/s (%d%%) %lums\r\n",
               cycle, vel,
               (int)((int32_t)vel * 100 / (int32_t)(ROBOT_MAX_VEL * 1000.0f)),
               (unsigned long)elapsed);
        state = ST_APPROACH;
        Timer_Reset(&timer);
        break;
      }

      if (elapsed > HOLDTEST_MAX_RETREAT_MS) {
        // 上限まで保持できた。走りすぎ防止で一旦前進へ戻す
        printf("[HOLDTEST] サイクル%u 上限%dmm/sまで保持 -> 前進へ\r\n", cycle,
               kMaxRetreatMmps);
        state = ST_APPROACH;
        Timer_Reset(&timer);
        break;
      }

      // 0 -> 最大 へ線形に加速。後退なので -x
      int16_t vel = (elapsed >= HOLDTEST_RETREAT_RAMP_MS)
                        ? kMaxRetreatMmps
                        : (int16_t)((int32_t)kMaxRetreatMmps * elapsed /
                                    HOLDTEST_RETREAT_RAMP_MS);
      OmniDrive_SetVel(&robot->omni_drive, -vel, 0, 0);
      break;
    }

    case ST_APPROACH:
      if (ball) {
        // 再捕捉。後退ランプは 0 からやり直す
        cycle++;
        state = ST_RETREAT;
        Timer_Reset(&timer);
        printf("[HOLDTEST] 再捕捉 -> サイクル%u 後退ランプを0から再開\r\n",
               cycle);
        break;
      }
      if (Timer_ReadMs(&timer) > HOLDTEST_MAX_APPROACH_MS) {
        printf("[HOLDTEST] 前進しても拾えないため停止。ボールを置き直してください\r\n");
        state = ST_IDLE;
        Timer_Reset(&timer);
        break;
      }
      // ゆっくり前進してボールを拾いに行く
      OmniDrive_SetVel(&robot->omni_drive, HOLDTEST_APPROACH_MMPS, 0, 0);
      break;

    case ST_IDLE:
    default:
      OmniDrive_SetVel(&robot->omni_drive, 0, 0, 0);
      if (ball) {
        cycle++;
        state = ST_RETREAT;
        Timer_Reset(&timer);
        printf("[HOLDTEST] ボール検知 -> サイクル%u 再開\r\n", cycle);
      }
      break;
  }
}
