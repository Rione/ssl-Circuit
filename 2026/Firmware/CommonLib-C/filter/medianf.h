#ifndef MEDIANF_H_
#define MEDIANF_H_

#include <stdint.h>

// 中央値フィルタ (Median Filter)

typedef struct {
      double* buffer;        // データを保持するリングバッファ（外部から渡す）
      uint16_t window_size;  // 窓幅（中央値を計算するサンプル数）
      uint16_t index;        // 現在書き込み位置
      uint16_t count;        // これまでに蓄積したサンプル数（window_size まで増加）
} MedianF;

// 中央値フィルタの初期化
// buffer は window_size 要素以上の配列を事前に確保して渡す
static inline void MedianF_Init(MedianF* mf, double* buffer, uint16_t window_size) {
      mf->buffer = buffer;
      mf->window_size = window_size;
      mf->index = 0;
      mf->count = 0;

      // バッファを 0 で初期化
      for (uint16_t i = 0; i < window_size; i++) {
            mf->buffer[i] = 0.0;
      }
}

// 新しいサンプルを追加し、中央値を返す
static inline double MedianF_Update(MedianF* mf, double new_val) {
      // バッファ未設定や窓幅 0 の場合は入力値をそのまま返す
      if (mf->buffer == NULL || mf->window_size == 0) {
            return new_val;
      }

      // 新しいサンプルをリングバッファに追加
      mf->buffer[mf->index] = new_val;
      mf->index++;
      if (mf->index >= mf->window_size) {
            mf->index = 0;
      }

      // サンプル数が窓幅に達するまでは count を増やす
      if (mf->count < mf->window_size) {
            mf->count++;
      }

      uint16_t n = mf->count;
      double tmp[n];

      // 現在の有効サンプルを一時配列にコピー
      for (uint16_t i = 0; i < n; i++) {
            tmp[i] = mf->buffer[i];
      }

      // 挿入ソートで昇順に並べ替え
      for (uint16_t i = 1; i < n; i++) {
            double key = tmp[i];
            int16_t j = (int16_t)i - 1;
            while (j >= 0 && tmp[j] > key) {
                  tmp[j + 1] = tmp[j];
                  j--;
            }
            tmp[j + 1] = key;
      }

      // 要素数が奇数なら中央の値，偶数なら中央2つの平均を返す
      if (n % 2 == 1) {
            return tmp[n / 2];
      } else {
            return (tmp[(n / 2) - 1] + tmp[n / 2]) * 0.5;
      }
}

#endif  // MEDIANF_H_
