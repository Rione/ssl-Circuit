#ifndef __BUZZER_H_
#define __BUZZER_H_

#include <stdint.h>

// MainBoard 上の圧電素子を、TIM1_CH1 (PA8) の PWM で鳴らす (docs/HANDOFF_ROCK5A_COMM.md: この PWM に素子が反応する)。
// ノンブロッキング: Buzzer_Play で音のパターンを選び、Buzzer_Update を制御周期ごとに呼ぶ。
typedef enum {
  BUZZER_NONE = 0,
  BUZZER_START,    // 短い1音 (自動最適化の開始)
  BUZZER_SUCCESS,  // 上がる3音 (保存まで完了)
  BUZZER_FAILURE,  // 低い長音を3回 (失敗・中止・安全停止)
} BuzzerPattern;

void Buzzer_Play(BuzzerPattern pattern);
void Buzzer_Update(void);  // 制御周期ごとに呼ぶ
void Buzzer_Stop(void);

#endif  // __BUZZER_H_
