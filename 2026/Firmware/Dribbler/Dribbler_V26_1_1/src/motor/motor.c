#include "motor.h"

PwmOut MD_INA;
PwmOut MD_INB;

void Motor_Init(void) {
      printf("Motor Init Start\n");
      PwmOut_Init(&MD_INA, &htim3, TIM_CHANNEL_4);
      PwmOut_Init(&MD_INB, &htim3, TIM_CHANNEL_4);
}
