#include "main_mode.h"

void MainMode_Init(MainMode *self, Robot *robot) {
  self->robot = robot;
  LocalController_Init(&self->local_controller);
}

void MainMode_Loop(MainMode *self) {
  static Timer timer = {0};
  Timer_Reset(&timer);

  Robot *r = self->robot;

  // シリアル通信
  Robot_RockRecvSerial(r, &r->info);

  // UIからの操作を更新
  Robot_UpdateFromUi(r);

  Robot_RockSendSerial(r, &r->info);
  OmniDrive_Recv(&r->omni_drive);

  if (!r->info.status.emergency_stop && r->info.status.is_signal_received) {
    // Robot is Running
    Robot_SendDribble(r, r->info.dribble_power, 0);
    Robot_SendKicker(r, &r->info);

    if (r->info.status.do_charge) {
      Kicker_Charge(&r->kicker);
    } else {
      Kicker_Discharge(&r->kicker);
    }
    Robot_SendOmniDrive(r, &r->info, 5);  // 5ms ごとに送信
  } else {
    // Robot is Stop or Emergency Stop
    LocalController_Stop(&self->local_controller, r);
  }

  Robot_UpdateHeartBeat(r);

  // 動作確認用 LED2 Lチカ（500ms間隔）
  static uint32_t led2_count = 0;
  static uint8_t led2_state = 0;
  led2_count++;
  if (led2_count >= 500) {
    led2_count = 0;
    led2_state ^= 1;
    DigitalOut_Write(&r->led2, led2_state);
  }

  // 1ms 制御ループを維持
  while (Timer_ReadMs(&timer) < 1);
}
