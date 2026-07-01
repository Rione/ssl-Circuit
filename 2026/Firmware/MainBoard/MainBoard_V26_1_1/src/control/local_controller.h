#ifndef __LOCAL_CONTROLLER_H_
#define __LOCAL_CONTROLLER_H_

#include "robot.h"

typedef struct {
  int dummy;  // C では空の struct は未定義動作のため
} LocalController;

void LocalController_Init(LocalController *self);
void LocalController_Stop(LocalController *self, Robot *robot);
void LocalController_TestMove(LocalController *self, Robot *robot);

#endif  // __LOCAL_CONTROLLER_H_
