#ifndef __MAIN_MODE_H_
#define __MAIN_MODE_H_

#include "local_controller.h"
#include "parammeter.h"
#include "robot.h"
#include "timer.h"

typedef struct {
  Robot           *robot;
  LocalController  local_controller;
} MainMode;

void MainMode_Init(MainMode *self, Robot *robot);
void MainMode_Loop(MainMode *self);

#endif  // __MAIN_MODE_H_
