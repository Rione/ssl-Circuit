#include "kicker.h"

#define KICK_TIME_MS 10         // キック通電時間 [ms]
#define CHARGE_RESET_US 200     // CHARGEをトグルしてラッチを解除する時間 [us]
#define KICK_READY_VOLTAGE 250  // この昇圧電圧[V]以上でのみキックを許可
#define KICK_MAX_DUTY 0.9f      // キックPWMデューティの上限

PwmOut kick1;
PwmOut kick2;

DigitalIn lt_done;

DigitalOut lt_charge;
DigitalOut lt_discharge;

Timer kick_timer;              // キックパルスの経過時間計測用
volatile bool is_kicking;      // キックパルス出力中フラグ
volatile bool is_discharging;  // 放電中フラグ

float boost_voltage;  // 最新の昇圧電圧 [V](app側からSetで更新)

void Kicker_SetBoostVoltage(float voltage) { boost_voltage = voltage; }

void Kicker_Init() {
  PwmOut_Init(&kick1, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&kick2, &htim2, TIM_CHANNEL_3);

  DigitalOut_Init(&lt_charge, LT_CHARGE_GPIO_Port, LT_CHARGE_Pin);
  DigitalOut_Init(&lt_discharge, LT_DISCHARGE_GPIO_Port, LT_DISCHARGE_Pin);
  DigitalIn_Init(&lt_done, LT_DONE_GPIO_Port, LT_DONE_Pin);

  Timer_Init(&kick_timer);
  is_kicking = false;
  is_discharging = false;
}

// ISRから呼ばれる。待たずにパルスを開始するだけ。
void Kicker_Kick(int kickType, float power) {
  // 昇圧電圧が十分なときのみキックする(DONEはFAULTと区別できないため電圧で判定)
  if (boost_voltage < KICK_READY_VOLTAGE) return;

  DigitalOut_Write(&lt_charge, 0);
  DigitalOut_Write(&lt_discharge, 0);
  is_discharging = false;

  // 強さはPWMのデューティ(power)で制御する。電流制限のため上限でクランプ。
  float duty = (power > KICK_MAX_DUTY) ? KICK_MAX_DUTY : power;
  switch (kickType) {
    case 1:  // ストレートキック
      PwmOut_Write(&kick1, duty);
      break;
    case 2:  // チップキック
      PwmOut_Write(&kick2, duty);
      break;
    default:
      return;
  }

  Timer_Reset(&kick_timer);
  is_kicking = true;
}

// メインループから毎周期呼ぶ。規定時間が経過したらキック出力をOFFにする。
void Kicker_Update() {
  if (is_kicking && Timer_ReadMs(&kick_timer) >= KICK_TIME_MS) {
    is_kicking = false;

    Kicker_Charge();  // キック後は自動で再チャージを開始
  }
  if (is_kicking == false) {
    if (is_discharging) {
      PwmOut_Write(&kick1, 0.025f);  // 放電中は継続して出力し続ける
      PwmOut_Write(&kick2, 0);
    } else {
      PwmOut_Write(&kick1, 0);
      PwmOut_Write(&kick2, 0);
    }
  }
}

void Kicker_Charge() {
  printf("Charge\n");
  DigitalOut_Write(&lt_discharge, 0);
  is_discharging = false;

  // CHARGEを一度確実にLowにしてからHighにし、Low->Highのトグルで
  // LT3751のラッチ(DONE/FAULT)を解除して新しい充電サイクルを開始する。
  DigitalOut_Write(&lt_charge, 0);
  WaitUs(CHARGE_RESET_US);
  DigitalOut_Write(&lt_charge, 1);
}

void Kicker_Discharge() {
  printf("Discharge\n");
  is_discharging = true;
  DigitalOut_Write(&lt_charge, 0);
  DigitalOut_Write(&lt_discharge, 1);
}

bool Kicker_DoneCheck() {
  // DONEピンはオープンコレクタのアクティブLow(完了でLow、未完了でHigh)
  bool done = !DigitalIn_Read(&lt_done);
  return done;
}