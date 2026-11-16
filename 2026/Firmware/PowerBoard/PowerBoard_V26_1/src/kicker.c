#include "kicker.h"

#define KICK_TIME_MS 10          // キック通電時間 [ms]
#define CHARGE_RESET_US 200      // CHARGEをトグルしてラッチを解除する時間 [us]
#define KICK_READY_VOLTAGE 150   // この昇圧電圧[V]以上でのみキックを許可
#define KICK_MAX_DUTY 0.95f      // キックPWMデューティの上限
#define DISCHARGE_DUTY 0.1f      // 放電中にソレノイドへ出すduty
#define DISCHARGE_TOGGLE_MS 250  // 放電中にduty ON/OFFを切り替える周期 [ms](ビヨンビヨン動作)
#define KICK_INTERVAL_MS 1000    // ダイレクトキックの最小間隔 [ms](MainBoardのROBOT_KICK_INTERVAL_MSと同じ)
#define DIRECT_KICK_TIMEOUT_MS 300  // 待機指示がこの時間更新されなければ待機を解除 [ms]

PwmOut kick1;
PwmOut kick2;

DigitalIn lt_done;

DigitalOut lt_charge;
DigitalOut lt_discharge;

Timer kick_timer;              // キックパルスの経過時間計測用
Timer discharge_timer;         // 放電ON/OFFトグルの経過時間計測用
volatile bool is_kicking;      // キックパルス出力中フラグ
volatile bool is_discharging;  // 放電中フラグ

// CANで受けたチャージ/放電要求。ISRが後勝ちで上書きし、Kicker_Updateで消費する。
typedef enum { KICKER_REQ_NONE, KICKER_REQ_CHARGE, KICKER_REQ_DISCHARGE } KickerReq;
volatile KickerReq kicker_req;
KickerReq kicker_mode;  // 最後に反映したチャージ/放電指示(キック後はこれに戻す)

float boost_voltage;  // 最新の昇圧電圧 [V](app側からSetで更新)

// ダイレクトキック待機状態(CAN受信ISRのみで読み書きする)。
// 時刻はHAL_GetTick()[ms]で扱う(DWTのTimerは数十秒で一周し、古い待機指示を有効と誤判定するため)
static uint8_t direct_kick_type;      // 0=解除, 1=ストレート, 2=チップ
static uint8_t direct_kick_power;     // 0-255
static uint32_t direct_kick_arm_tick;  // 最後に待機指示を受けた時刻 [ms]
static bool prev_photo_detected;      // フォトセンサの前回値(立ち上がり検出用)
static uint32_t last_kick_tick;       // 最後にキックした時刻 [ms]
static bool has_kicked;               // 起動後に一度でもキックしたか

void Kicker_SetBoostVoltage(float voltage) { boost_voltage = voltage; }

void Kicker_Init() {
  PwmOut_Init(&kick1, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&kick2, &htim2, TIM_CHANNEL_3);

  DigitalOut_Init(&lt_charge, LT_CHARGE_GPIO_Port, LT_CHARGE_Pin);
  DigitalOut_Init(&lt_discharge, LT_DISCHARGE_GPIO_Port, LT_DISCHARGE_Pin);
  DigitalIn_Init(&lt_done, LT_DONE_GPIO_Port, LT_DONE_Pin);

  Timer_Init(&kick_timer);
  Timer_Init(&discharge_timer);
  is_kicking = false;
  is_discharging = false;
  kicker_req = KICKER_REQ_NONE;
  kicker_mode = KICKER_REQ_NONE;

  direct_kick_type = 0;
  direct_kick_power = 0;
  prev_photo_detected = false;
  has_kicked = false;
}

// ISRから呼ばれる。待たずにパルスを開始するだけ。キックした場合はtrueを返す。
bool Kicker_Kick(int kickType, float power) {
  if (kickType != 1 && kickType != 2) return false;
  // 昇圧電圧が十分なときのみキックする(DONEはFAULTと区別できないため電圧で判定)
  if (boost_voltage < KICK_READY_VOLTAGE) return false;

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
  }

  Timer_Reset(&kick_timer);
  is_kicking = true;
  last_kick_tick = HAL_GetTick();
  has_kicked = true;
  return true;
}

// ISRから呼ばれる。MainBoardからの待機指示を記録する。
void Kicker_SetDirectKick(uint8_t type, uint8_t power) {
  direct_kick_type = (type == 1 || type == 2) ? type : 0;
  direct_kick_power = power;
  direct_kick_arm_tick = HAL_GetTick();
}

// ISRから呼ばれる。Dribblerのフォトセンサ値を受け、待機中に立ち上がりを検出したら即キックする。
void Kicker_OnPhotoSensor(bool detected) {
  bool rising_edge = detected && !prev_photo_detected;
  prev_photo_detected = detected;
  if (!rising_edge) return;

  uint32_t now = HAL_GetTick();
  if (direct_kick_type == 0 || direct_kick_power == 0) return;
  if (now - direct_kick_arm_tick > DIRECT_KICK_TIMEOUT_MS) return;  // MainBoardからの指示が途絶えた
  if (has_kicked && now - last_kick_tick < KICK_INTERVAL_MS) return;

  Kicker_Kick(direct_kick_type, direct_kick_power / 255.0f);
}

// ISRから呼ばれる。要求を記録するだけで、実際の切り替えはKicker_Updateで行う。
void Kicker_RequestCharge() { kicker_req = KICKER_REQ_CHARGE; }
void Kicker_RequestDischarge() { kicker_req = KICKER_REQ_DISCHARGE; }

// 充電を開始する。待ちを含むためメインループからのみ呼ぶ。
static void Kicker_Charge() {
  printf("Charge\n");
  DigitalOut_Write(&lt_discharge, 0);
  WaitUs(1000);

  // CHARGEを一度確実にLowにしてからHighにし、Low->Highのトグルで
  // LT3751のラッチ(DONE/FAULT)を解除して新しい充電サイクルを開始する。
  DigitalOut_Write(&lt_charge, 0);
  WaitUs(CHARGE_RESET_US);

  // 待ちの間にキックが割り込んだ場合は充電を開始しない(キック後に自動で再チャージされる)
  __disable_irq();
  if (!is_kicking) DigitalOut_Write(&lt_charge, 1);
  __enable_irq();
}

// メインループから毎周期呼ぶ。キックパルスの終了と、チャージ/放電要求の反映を行う。
// ISR(Kicker_Kick)と出力ピンを取り合わないよう、判定と出力は割り込み禁止区間で行う。
void Kicker_Update() {
  KickerReq req = KICKER_REQ_NONE;

  __disable_irq();
  if (is_kicking && Timer_ReadMs(&kick_timer) >= KICK_TIME_MS) {
    is_kicking = false;
    // キック後はキック前の指示(充電/放電)に戻す(キック中に届いた要求があればそちらを優先)
    if (kicker_req == KICKER_REQ_NONE) kicker_req = kicker_mode;
  }
  if (!is_kicking) {  // キック中の要求は保留し、キック終了後に反映する
    req = kicker_req;
    kicker_req = KICKER_REQ_NONE;
    if (req != KICKER_REQ_NONE) kicker_mode = req;

    if (req == KICKER_REQ_CHARGE) {
      is_discharging = false;
    } else if (req == KICKER_REQ_DISCHARGE) {
      if (!is_discharging) Timer_Reset(&discharge_timer);  // 放電開始時のみ周期をリセット
      is_discharging = true;
      DigitalOut_Write(&lt_charge, 0);
      DigitalOut_Write(&lt_discharge, 1);
    }

    if (is_discharging) {
      // DISCHARGE_TOGGLE_MSごとにduty ON/OFFを切り替え、ソレノイドを
      // ビヨンビヨンと振動させながら放電する。
      uint32_t elapsed = Timer_ReadMs(&discharge_timer);
      bool toggle_on = (elapsed / DISCHARGE_TOGGLE_MS) % 2 == 0;
      PwmOut_Write(&kick1, toggle_on ? DISCHARGE_DUTY : 0);
      PwmOut_Write(&kick2, 0);
    } else {
      PwmOut_Write(&kick1, 0);
      PwmOut_Write(&kick2, 0);
    }
  }
  __enable_irq();

  if (req == KICKER_REQ_CHARGE) {
    Kicker_Charge();
  } else if (req == KICKER_REQ_DISCHARGE) {
    printf("Discharge\n");
  }
}

bool Kicker_DoneCheck() {
  // DONEピンはオープンコレクタのアクティブLow(完了でLow、未完了でHigh)
  bool done = !DigitalIn_Read(&lt_done);
  return done;
}