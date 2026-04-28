#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

static inline void BLDC_WritePwm(double u, double v, double w) {
  u = Constrain(u, MIN_DUTY, MAX_DUTY);
  v = Constrain(v, MIN_DUTY, MAX_DUTY);
  w = Constrain(w, MIN_DUTY, MAX_DUTY);

  PwmOut_Write(&u_pwm, u);
  PwmOut_Write(&v_pwm, v);
  PwmOut_Write(&w_pwm, w);
}

void BLDC_Init() {
  printf("BLDC_Init\n");
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);
}

void BLDC_OpenLoopDrive(double amp, double phase) {
  phase = NormalizeRadians(phase);

  double u = 0.5 + 0.5 * amp * Cos(phase);
  double v = 0.5 + 0.5 * amp * Cos(phase - TWO_THIRDS_PI);
  double w = 0.5 + 0.5 * amp * Cos(phase + TWO_THIRDS_PI);

  BLDC_WritePwm(u, v, w);
}