#ifndef __LOCAL_CONTROLLER_H_
#define __LOCAL_CONTROLLER_H_

#include "robot.h"

typedef struct {
  int dummy;  // C では空の struct は未定義動作のため
} LocalController;

void LocalController_Init(LocalController *self);
void LocalController_Stop(LocalController *self, Robot *robot);
void LocalController_TestMove(LocalController *self, Robot *robot);
void LocalController_TestMoveForwardBack(LocalController *self, Robot *robot);
void LocalController_TestTCSAcceleration(LocalController *self, Robot *robot);
void LocalController_TestWheelSpin(LocalController *self, Robot *robot);
void LocalController_TestVoltage(LocalController *self, Robot *robot);
void LocalController_TestVoltageFF(LocalController *self, Robot *robot);
void LocalController_TestMotionPattern(LocalController *self, Robot *robot);

typedef enum {
  MOTION_PATTERN_RUNNING = 0,   // 待機中・走行中
  MOTION_PATTERN_FINISHED = 1,  // 全区間を走り終えた
  MOTION_PATTERN_ABORTED = 2,   // 安全停止で終わった
} MotionPatternStatus;

// 動作パターンのテストを最初からやり直せるようにする。startup_wait_ms: 最初に呼ばれてから走り出すまで
// の待ち時間、dump_log: 終了60秒後に記録を UART へ出力するか (既定は 10000ms・出力する)
void LocalController_ResetMotionPattern(uint32_t startup_wait_ms, bool dump_log);
MotionPatternStatus LocalController_GetMotionPatternStatus(void);

#endif  // __LOCAL_CONTROLLER_H_
