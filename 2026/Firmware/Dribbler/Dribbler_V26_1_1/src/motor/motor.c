#include "motor.h"

PwmOut MD_INA;
PwmOut MD_INB;

void Motor_Init(void) {
      printf("Motor Init Start\n");
      PwmOut_Init(&MD_INA, &htim3, TIM_CHANNEL_4);
      PwmOut_Init(&MD_INB, &htim3, TIM_CHANNEL_3);
}

void Motor_Drive(float duty) {
      if (duty >= 0) {
            PwmOut_Write(&MD_INA, duty);
            PwmOut_Write(&MD_INB, 0);
      } else {
            PwmOut_Write(&MD_INA, 0);
            PwmOut_Write(&MD_INB, -duty);
      }
}

void Motor_Brake() {
      PwmOut_Write(&MD_INA, 1.0f);
      PwmOut_Write(&MD_INB, 1.0f);
}
