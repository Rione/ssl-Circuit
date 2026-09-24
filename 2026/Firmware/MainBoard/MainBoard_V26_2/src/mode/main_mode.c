#include "main_mode.h"

#include <stdio.h>

#include "auto_tune.h"
#include "iwdg.h"

Timer main_control_timer;

void MainMode_Init(MainMode* self, Robot* robot) {
  self->robot = robot;
  LocalController_Init(&self->local_controller);
  Timer_Init(&main_control_timer);
#if AUTOTUNE_IGNORE_ROCK_COMMANDS
  printf("# WORKAROUND: AUTOTUNE_IGNORE_ROCK_COMMANDS=1 (Rock5A commands are ignored)\n");
#endif

  Kicker_Discharge(&self->robot->kicker);
}

void MainMode_Loop(MainMode* self) {
  Robot* r = self->robot;

  Robot_UpdateSensor(r);

  // シリアル通信
  Robot_UpdateFromUi(r);

  Robot_RockUpdateSPI(r, &r->info);

  OmniDrive_Recv(&r->omni_drive);

  // 自動チューニングの開発中は、Rock5A の指令を受け付けない (AUTOTUNE_IGNORE_ROCK_COMMANDS)。
  // Rock5A を付けたままでも、ST-Link から指示したテストを邪魔されない
  const bool rock_commands_enabled = (AUTOTUNE_IGNORE_ROCK_COMMANDS == 0);
  if (rock_commands_enabled && !r->info.status.emergency_stop &&
      r->info.status.is_signal_received) {
    // Robot is Running
    Robot_SendDribble(r, r->info.dribble_power, 0);
    Robot_SendKicker(r, &r->info);

    if (r->info.status.do_charge) {
      Kicker_Charge(&r->kicker);
    } else {
      Kicker_Discharge(&r->kicker);
    }
    Robot_SendOmniDrive(r, &r->info, 1);  // 1ms ごとに送信
    AutoTune_Cancel();  // Rock5A の指令を受けたら、ST-Link から指示したテストは取り消す
  } else {
    // Robot is Stop or Emergency Stop
    // ST-Link から開始を指示したときだけテストを走らせる (src/control/auto_tune.h、tools/autotune_start.ps1)。
    // 指示が無ければ止まる。Rock5A から緊急停止を受けているときは指示を取り消して止まる。
    // ただし Rock5A の信号が来ていない間は emergency_stop が常に1になっていて、テストが始まる前に
    // 取り消されたため、信号を受信していて緊急停止が1のときだけ取り消す (実際の緊急停止は見続ける)
    if (r->info.status.emergency_stop && r->info.status.is_signal_received) {
      AutoTune_Cancel();
      LocalController_Stop(&self->local_controller, r);
    } else if (!AutoTune_Poll(&self->local_controller, r)) {
      LocalController_Stop(&self->local_controller, r);
    }
    // LocalController_TestMove(&self->local_controller, r);
    // LocalController_TestMoveForwardBack(&self->local_controller, r);

    // ★ TCS性能検証用テスト (電源投入後10秒待機、2000mm/s最速加速、1.5m前後往復)
    // (出力を電圧制御にするかは parammeter.h の TEST_TCS_USE_VOLTAGE_CONTROL)
    // LocalController_TestTCSAcceleration(&self->local_controller, r);

    // ★ 動作パターンのテスト (床で実施、電圧制御。前後・左右・斜め・旋回。原点から x: -0.5〜1.5m, y: ±1.5m)
    // ⚠ Rock5A接続中に信号が途切れてもこのテストが動く。試すときだけ上のLocalController_Stopをコメントアウトし、
    //    このテストのコメントを外すこと (このコミットではLocalController_Stopが有効)
    // LocalController_TestMotionPattern(&self->local_controller, r);

    // ★ WheelUnit ID・回転方向・受信確認用テスト (機体を浮かせて実施)
    // LocalController_TestWheelSpin(&self->local_controller, r);

    // ★ 電圧モードの単体評価用テスト (機体を浮かせて実施。4輪に最大4V)
    // LocalController_TestVoltage(&self->local_controller, r);

    // ★ 電圧制御 (フィードフォワードのみ) の確認用テスト (機体を浮かせて実施。前後±2m/s・左右・旋回)
    // LocalController_TestVoltageFF(&self->local_controller, r);
  }

  Robot_UpdateHeartBeat(r);
  HAL_IWDG_Refresh(&hiwdg);

  if (r->info.kicker_status.cap_val > 100) {
    DigitalOut_Write(&r->led1, 1);
  } else {
    DigitalOut_Write(&r->led1, 0);
  }

  // if (r->info.status.do_direct_straight || r->info.status.do_direct_chip) {
  //   DigitalOut_Write(&r->led2, 1);
  // } else {
  //   DigitalOut_Write(&r->led2, 0);
  // }

  if (r->info.status.is_signal_received) {
    DigitalOut_Write(&r->led2, 1);
  } else {
    DigitalOut_Write(&r->led2, 0);
  }

  while (Timer_ReadUs(&main_control_timer) < ROBOT_CONTROL_LOOP_DT_US) {
    // 制御ループ周期まで待機
  };
  Timer_Reset(&main_control_timer);
}
