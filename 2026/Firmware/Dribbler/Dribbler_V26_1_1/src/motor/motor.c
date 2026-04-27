#include "motor.h"
#include <stdint.h>

PwmOut MD_INA;
PwmOut MD_INB;

MAF maf;
static uint16_t filtered_current = 0;

static uint32_t base_currents[MAX_SPEED_LEVEL + 1];
static uint8_t current_drive_level = 0;

void Motor_Init() {
  PwmOut_Init(&MD_INA, &htim3, TIM_CHANNEL_4);
  PwmOut_Init(&MD_INB, &htim3, TIM_CHANNEL_3);

  MAF_Init(&maf, 50);
}

bool Motor_SetBaseCurrent(uint16_t current_val) {
  static uint16_t count = 0;
  static uint8_t measure_level = 1;

  if (measure_level > MAX_SPEED_LEVEL) {
    Motor_Free(); // すべての計測が終わったらモータを止める
    return true;  // キャリブレーション完了
  }

  // 最初にモータをそのレベルで回す
  if (count == 0) {
    Motor_Drive(measure_level);
  }

  // モータが定常状態になるまで（100サンプル）待ってから加算する
  if (count > 100) {
    base_currents[measure_level] += current_val;
  }

  count++;

  // BASE_CURRENT_MEASURE_NUMに達したら平均を計算して次のレベルへ
  if (count >= BASE_CURRENT_MEASURE_NUM) {
    // 蓄積したサンプル数で割る
    base_currents[measure_level] /= (BASE_CURRENT_MEASURE_NUM - 100);

    measure_level++;
    count = 0;
  }

  return false;
}

void Motor_Drive(uint8_t level) {
  if (level > MAX_SPEED_LEVEL) {
    level = MAX_SPEED_LEVEL;
  }
  current_drive_level = level;

  float duty = (float)level / MAX_SPEED_LEVEL;

  if (duty >= 0) {
    PwmOut_Write(&MD_INA, duty);
    PwmOut_Write(&MD_INB, 0);
  } else {
    PwmOut_Write(&MD_INA, 0);
    PwmOut_Write(&MD_INB, -duty);
  }
}

void Motor_Brake() {
  current_drive_level = 0;
  PwmOut_Write(&MD_INA, 1.0f);
  PwmOut_Write(&MD_INB, 1.0f);
}

void Motor_Free() {
  current_drive_level = 0;
  PwmOut_Write(&MD_INA, 0);
  PwmOut_Write(&MD_INB, 0);
}

void Motor_Update(uint16_t current_val) {
  filtered_current = MAF_Update(&maf, current_val);
}

bool Motor_IsBallCaptured() {
  // モータ停止中は判定しない
  if (current_drive_level == 0) {
    return false;
  }

  // 現在のスピードレベルに対応するベース電流を取得
  uint32_t expected_base = base_currents[current_drive_level];

  // ベース電流 + 閾値 を超えていればボールを捕捉
  if (filtered_current > expected_base + MOTOR_CURRENT_THRESHOLD) {
    return true;
  }

  return false;
}