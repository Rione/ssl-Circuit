#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

void BLDC_Init() {
  printf("BLDC_Init\n");
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);
}