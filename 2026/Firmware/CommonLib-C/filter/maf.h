#ifndef MAF_H_
#define MAF_H_

#include <stdint.h>

// 移動平均フィルタ (Moving Average Filter)

#define MAF_MAX_WINDOW 100 // 最大サイズを固定

typedef struct {
  double buffer[MAF_MAX_WINDOW]; // 構造体の中に実体を持つ
  uint16_t window_size;
  uint16_t index;
  uint16_t count;
  double sum;
} MAF;

// 移動平均フィルタの初期化
// window_size で指定したサンプル数分のバッファを動的確保し、0 クリアする
static inline void MAF_Init(MAF *maf, uint16_t window_size) {
  if (window_size > MAF_MAX_WINDOW) {
    window_size = MAF_MAX_WINDOW;
  }
  if (window_size == 0) {
    window_size = 1;
  }
  maf->window_size = window_size;
  maf->index = 0;
  maf->count = 0;
  maf->sum = 0.0;
  for (int i = 0; i < MAF_MAX_WINDOW; i++) {
    maf->buffer[i] = 0.0;
  }
}

// 新しいサンプルを追加し、移動平均値を返す
static inline double MAF_Update(MAF *maf, double new_val) {
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

#endif // MAF_H_
