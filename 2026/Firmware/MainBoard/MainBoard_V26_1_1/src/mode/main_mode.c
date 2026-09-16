#include "main_mode.h"

#include <stdio.h>

// ===========================================================================
// 試験用ビルド設定
// ===========================================================================
// 1: ドリブラ保持力・干渉チェック試験モード。
//    コントローラからの指令を無視し、ボール保持中に後退速度を上げていく。
//    ドリブラ側には FW/Dribbler_V26_1_1-holdtest を書き込んでおくこと。
// 0: 通常動作
#define MAINBOARD_HOLD_TEST 1

Timer main_control_timer;

void MainMode_Init(MainMode* self, Robot* robot) {
  self->robot = robot;
  LocalController_Init(&self->local_controller);
  Timer_Init(&main_control_timer);

  Kicker_Discharge(&self->robot->kicker);
}

void MainMode_Loop(MainMode* self) {
  Robot* r = self->robot;

  Robot_UpdateSensor(r);

  // シリアル通信
  Robot_UpdateFromUi(r);

  Robot_RockUpdateSPI(r, &r->info);

  OmniDrive_Recv(&r->omni_drive);

#if MAINBOARD_HOLD_TEST
  // 試験モード: コントローラ信号の有無によらず、ボール検知で動き出す。
  // 電源投入直後は停止しており、最初にボールを置くまで動かない。
  LocalController_TestBallHold(&self->local_controller, r);
#else
  if (!r->info.status.emergency_stop && r->info.status.is_signal_received) {
    // Robot is Running
    Robot_SendDribble(r, r->info.dribble_power, 0);
    Robot_SendKicker(r, &r->info);

    if (r->info.status.do_charge) {
      Kicker_Charge(&r->kicker);
    } else {
      Kicker_Discharge(&r->kicker);
    }
    Robot_SendOmniDrive(r, &r->info, 1);  // 1ms ごとに送信
  } else {
    // Robot is Stop or Emergency Stop
    LocalController_Stop(&self->local_controller, r);
    // LocalController_TestMove(&self->local_controller, r);
    // LocalController_TestMoveForwardBack(&self->local_controller, r);
  }
#endif  // MAINBOARD_HOLD_TEST

  Robot_UpdateHeartBeat(r);

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
