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

// ドリブラ保持力・干渉チェック試験。
// ボール保持中は後退速度を徐々に上げ、外れたらゆっくり前進して拾い直す。
void LocalController_TestBallHold(LocalController *self, Robot *robot);

#endif  // __LOCAL_CONTROLLER_H_
