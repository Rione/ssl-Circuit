#include "kicker.h"

PwmOut kick1;
PwmOut kick2;

DigitalIn lt_done;

DigitalOut lt_charge;
DigitalOut lt_discharge;

void Kicker_Init() {
  PwmOut_Init(&kick1, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&kick2, &htim2, TIM_CHANNEL_3);

  DigitalOut_Init(&lt_charge, LT_CHARGE_GPIO_Port, LT_CHARGE_Pin);
  DigitalOut_Init(&lt_discharge, LT_DISCHARGE_GPIO_Port, LT_DISCHARGE_Pin);
  DigitalIn_Init(&lt_done, LT_DONE_GPIO_Port, LT_DONE_Pin);
}

void Kicker_Kick(int kickType, float power) {
  DigitalOut_Write(&lt_charge, 0);
  DigitalOut_Write(&lt_discharge, 0);

  switch (kickType) {
    case 1:  // ストレートキック
      PwmOut_Write(&kick1, power);
      break;
    case 2:  // チップキック
      PwmOut_Write(&kick2, power);
      break;
    default:
      break;
  }
}

void Kicker_Charge() {
  printf("Charge\n");
  DigitalOut_Write(&lt_charge, 1);
  DigitalOut_Write(&lt_discharge, 0);
}

void Kicker_Discharge() {
  printf("Discharge\n");
  DigitalOut_Write(&lt_charge, 0);
  DigitalOut_Write(&lt_discharge, 1);
}

bool Kicker_DoneCheck() {
  bool done = DigitalIn_Read(&lt_done);
  if (done) {
    DigitalOut_Write(&lt_charge, 0);
    DigitalOut_Write(&lt_discharge, 0);
  }
  return done;
}