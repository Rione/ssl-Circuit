#ifndef __TCS_LOG_H_
#define __TCS_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "omni_drive.h"

// TCS実機チューニング用 RAMロガー
// 走行中は RAM に記録するだけ (1msループを崩さない)。停止後に TcsLog_DumpStep を
// 毎ループ呼ぶと、1回につき1行ずつ printf(USART1) で CSV を出力する。

#define TCS_LOG_MAX_SAMPLES 1000U  // 10ms周期で10秒分

void TcsLog_Reset(void);
// omni_drive の直近の TCS 状態を1サンプル記録する (満杯なら何もしない)
void TcsLog_Record(uint32_t t_ms, bool tcs_on, int16_t target_vx_mmps, float gyro_yaw_rate,
                   const OmniDrive* omni_drive);
// CSV を1行出力する。全行出力し終えたら true を返す
bool TcsLog_DumpStep(void);

#endif  // __TCS_LOG_H_
