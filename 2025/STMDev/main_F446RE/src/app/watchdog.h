#ifndef __WATCHDOG__
#define __WATCHDOG__

#include <stdint.h>

// 独立ウォッチドッグ(IWDG)。メインループが止まる(HardFault・無限ループ等)と自動でリセットする

// 今回の起動がウォッチドッグによるリセットかを判定する。リセット要因フラグを消すので起動直後に一度だけ呼ぶこと
void watchdogCheckResetCause();
bool watchdogCausedReset();
uint32_t watchdogResetCount();  // ウォッチドッグによるリセットの累計回数(電源を切ると0に戻る)

void watchdogStart(uint32_t timeoutMs);
void watchdogRefresh();

#endif
