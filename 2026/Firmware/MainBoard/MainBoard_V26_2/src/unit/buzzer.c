#include "buzzer.h"

#include <stddef.h>

#include "main.h"
#include "tim.h"

#define BUZZER_TIMER_CLOCK_HZ 16800000U  // TIM1: PSC=9 (tim.c)

typedef struct {
  uint16_t freq_hz;  // 0: 無音
  uint16_t ms;
} Note;

static const Note kStart[] = {{2000, 150}, {0, 0}};
static const Note kSuccess[] = {{1500, 200}, {0, 60}, {2000, 200}, {0, 60}, {2500, 350}, {0, 0}};
static const Note kFailure[] = {{1000, 500}, {0, 200}, {1000, 500}, {0, 200}, {1000, 500}, {0, 0}};

static const Note* current = NULL;
static size_t index = 0;
static uint32_t note_start_tick = 0;
static int pwm_on = 0;

static void SetTone(uint16_t freq_hz) {
  if (freq_hz == 0) {
    if (pwm_on) HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    pwm_on = 0;
    return;
  }
  uint32_t arr = BUZZER_TIMER_CLOCK_HZ / freq_hz - 1U;
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1U) / 2U);  // デューティ 50%
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  if (!pwm_on) HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  pwm_on = 1;
}

static void StartNote(void) {
  SetTone(current[index].freq_hz);
  note_start_tick = HAL_GetTick();
}

void Buzzer_Stop(void) {
  current = NULL;
  SetTone(0);
}

void Buzzer_Play(BuzzerPattern pattern) {
  switch (pattern) {
    case BUZZER_START: current = kStart; break;
    case BUZZER_SUCCESS: current = kSuccess; break;
    case BUZZER_FAILURE: current = kFailure; break;
    default: Buzzer_Stop(); return;
  }
  index = 0;
  StartNote();
}

void Buzzer_Update(void) {
  if (current == NULL) return;
  if (HAL_GetTick() - note_start_tick < current[index].ms) return;
  index++;
  if (current[index].ms == 0) {  // 終端
    Buzzer_Stop();
    return;
  }
  StartNote();
}
