#include "auto_tune.h"

#include <stdio.h>

#include "ramp_test.h"
#include "volt_tune.h"

volatile AutoTuneCtrl autotune_ctrl;

// 開始の指示を受けてから走り出すまでの時間 [ms] (ケーブルを抜いて機体から離れる)
#define AUTOTUNE_START_WAIT_MS 10000U

static uint32_t wait_start_tick = 0;

static bool AutoTune_IsRequested(void) {
  return autotune_ctrl.magic == AUTOTUNE_CTRL_MAGIC &&
         autotune_ctrl.start_seq != autotune_ctrl.done_seq;
}

static void AutoTune_Finish(AutoTuneResult result) {
  // 上書きした値は、走り終わったら (取り消しても) 既定値に戻す。段階4で、合格した値を保存して使う
  VoltTune_SetDefaults(&volt_tune);
  autotune_ctrl.result = result;
  autotune_ctrl.state = AUTOTUNE_STATE_IDLE;
  autotune_ctrl.done_seq = autotune_ctrl.start_seq;
}

bool AutoTune_Poll(LocalController* lc, Robot* robot) {
  switch ((AutoTuneState)autotune_ctrl.state) {
    case AUTOTUNE_STATE_IDLE:
    default:
      autotune_ctrl.state = AUTOTUNE_STATE_IDLE;
      if (!AutoTune_IsRequested()) return false;
      if (autotune_ctrl.test_id != AUTOTUNE_TEST_MOTION_PATTERN &&
          autotune_ctrl.test_id != AUTOTUNE_TEST_RAMP) {
        printf("# autotune: bad test_id=%lu\n", (unsigned long)autotune_ctrl.test_id);
        AutoTune_Finish(AUTOTUNE_RESULT_BAD_TEST);
        return false;
      }
      // この1回だけの上書き (0 は上書きしない)。ST-Link から値を書き換えていてもよいように、安全範囲に収める
      VoltTune_SetDefaults(&volt_tune);
      if (autotune_ctrl.traction_x100 != 0) volt_tune.traction_limit_v = autotune_ctrl.traction_x100 * 0.01f;
      if (autotune_ctrl.max_accel_x100 != 0) volt_tune.max_accel = autotune_ctrl.max_accel_x100 * 0.01f;
      if (autotune_ctrl.max_ang_accel_x100 != 0)
        volt_tune.max_ang_accel = autotune_ctrl.max_ang_accel_x100 * 0.01f;
      if (VoltTune_Sanitize(&volt_tune)) printf("# autotune: volt_tune corrected\n");
      printf("# autotune: traction=%d max_accel=%d max_ang_accel=%d (x100)\n",
             (int)(volt_tune.traction_limit_v * 100.0f), (int)(volt_tune.max_accel * 100.0f),
             (int)(volt_tune.max_ang_accel * 100.0f));
      printf("# autotune: start requested (seq=%lu test=%lu), running in %u ms\n",
             (unsigned long)autotune_ctrl.start_seq, (unsigned long)autotune_ctrl.test_id,
             AUTOTUNE_START_WAIT_MS);
      wait_start_tick = HAL_GetTick();
      autotune_ctrl.result = AUTOTUNE_RESULT_NONE;
      autotune_ctrl.state = AUTOTUNE_STATE_WAITING;
      // fallthrough
    case AUTOTUNE_STATE_WAITING: {
      LocalController_Stop(lc, robot);
      uint32_t waited_ms = HAL_GetTick() - wait_start_tick;
      // テスト自身の待ち (0.5秒ごと) と見分けるため、速く点滅させる
      DigitalOut_Write(&robot->led0, (waited_ms / 125) % 2 == 0);
      if (waited_ms < AUTOTUNE_START_WAIT_MS) return true;
      if (autotune_ctrl.test_id == AUTOTUNE_TEST_RAMP) {
        RampTest_Reset();
      } else {
        LocalController_ResetMotionPattern(0, false);
      }
      autotune_ctrl.state = AUTOTUNE_STATE_RUNNING;
      return true;
    }
    case AUTOTUNE_STATE_RUNNING: {
      if (autotune_ctrl.test_id == AUTOTUNE_TEST_RAMP) {
        RampTestStatus rs = RampTest_Step(robot);
        if (rs == RAMP_RUNNING) return true;
        OmniDrive_SetFree(&robot->omni_drive);
        DigitalOut_Write(&robot->led0, 0);
        AutoTune_Finish(rs == RAMP_FINISHED ? AUTOTUNE_RESULT_FINISHED : AUTOTUNE_RESULT_ABORTED);
        printf("# autotune: done (result=%lu)\n", (unsigned long)autotune_ctrl.result);
        return true;
      }
      LocalController_TestMotionPattern(lc, robot);
      MotionPatternStatus status = LocalController_GetMotionPatternStatus();
      if (status == MOTION_PATTERN_RUNNING) return true;
      OmniDrive_SetFree(&robot->omni_drive);
      DigitalOut_Write(&robot->led0, 0);
      AutoTune_Finish(status == MOTION_PATTERN_FINISHED ? AUTOTUNE_RESULT_FINISHED
                                                        : AUTOTUNE_RESULT_ABORTED);
      printf("# autotune: done (result=%lu)\n", (unsigned long)autotune_ctrl.result);
      return true;
    }
  }
}

void AutoTune_Cancel(void) {
  if (autotune_ctrl.state != AUTOTUNE_STATE_IDLE || AutoTune_IsRequested()) {
    if (autotune_ctrl.state == AUTOTUNE_STATE_RUNNING && autotune_ctrl.test_id == AUTOTUNE_TEST_RAMP) {
      RampTest_Cancel();
    }
    AutoTune_Finish(AUTOTUNE_RESULT_CANCELLED);
  }
}
