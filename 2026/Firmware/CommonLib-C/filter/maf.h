#ifndef MAF_H_
#define MAF_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// 移動平均フィルタ (Moving Average Filter)

typedef struct {
      double* buffer;        // データを保持するリングバッファ
      uint16_t window_size;  // 窓幅（平均を取るサンプル数）
      uint16_t index;        // 現在書き込み位置
      uint16_t count;        // これまでに蓄積したサンプル数（window_size まで増加）
      double sum;            // バッファ内サンプルの合計値
} MAF;

// 移動平均フィルタの初期化
// window_size で指定したサンプル数分のバッファを動的確保し、0 クリアする
static inline void MAF_Init(MAF* maf, uint16_t window_size) {
      maf->window_size = window_size;
      maf->index = 0;
      maf->count = 0;
      maf->sum = 0.0;
      maf->buffer = (double*)malloc(sizeof(double) * window_size);
      if (maf->buffer) {
            memset(maf->buffer, 0, sizeof(double) * window_size);
      }
}

// 新しいサンプルを追加し、移動平均値を返す
static inline double MAF_Update(MAF* maf, double new_val) {
      // バッファ未確保や窓幅 0 の場合は入力値をそのまま返す
      if (maf->buffer == NULL || maf->window_size == 0) {
            return new_val;
      }

      // サンプル数が窓幅に達するまでは単純にカウントアップ
      if (maf->count < maf->window_size) {
            maf->count++;
      } else {
            // 古いサンプルを合計値から減算
            maf->sum -= maf->buffer[maf->index];
      }

      // 新しいサンプルを追加して合計値を更新
      maf->buffer[maf->index] = new_val;
      maf->sum += new_val;

      // 書き込みインデックスをリングバッファとして更新
      maf->index++;
      if (maf->index >= maf->window_size) {
            maf->index = 0;
      }

      // 現在の平均値を返す
      return maf->sum / (double)maf->count;
}

#endif  // MAF_H_
