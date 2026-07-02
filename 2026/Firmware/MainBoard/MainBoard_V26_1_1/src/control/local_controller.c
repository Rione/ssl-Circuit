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
