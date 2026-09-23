#ifndef __LMSInterpolator__
#define __LMSInterpolator__

#include "Timer.hpp"

class LMSInterpolator {
  public:
    LMSInterpolator() {}
    ~LMSInterpolator() {}

    // やりたいこと
    //  1. 時間と値のペアを作成する
    //  2. 係数を計算する
    //  3. 係数を使って時間tあたりの補間値を計算する

    // 時間と値のペアを入力する
    // 全部で3回のデータを保持する．今回の値の時間を0として，それ以前の値の時間を負の値で保持するように変換する必要がある
    void addSample(float value) {
        // push_backと同じく後ろに追加する
        float elapsedTime = timer.read_ms(); // 経過時間を取得
        sampleX[0] = sampleX[1] - elapsedTime;
        sampleX[1] = sampleX[2] - elapsedTime;
        sampleX[2] = 0; // elapsedTime - elapsedTimeで0になる

        sampleY[0] = sampleY[1];
        sampleY[1] = sampleY[2];
        sampleY[2] = value;

        calculateCoefficients();
    }

    // 係数を使って時間tあたりの補間値を計算する
    float getInterpolateValue() {
        float t = timer.read_ms();
        return coefficients[0][0] + coefficients[1][0] * t + coefficients[2][0] * t * t;
    }

  private:
    Timer timer;
    float sampleX[3];
    float sampleY[3];
    float coefficients[3][1];

    // forを使わないで行列の積を計算する関数
    void multiplyMatrices(float a[3][3], float b[3][1], float result[3][1]) {
        result[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
        result[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
        result[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
    }

    // 逆行列を計算する関数（クラメルの公式を使用）
    bool invertMatrix(float a[3][3], float result[3][3]) {
        float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if (det == 0.0) return false;

        float invDet = 1.0 / det;
        result[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * invDet;
        result[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invDet;
        result[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invDet;
        result[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * invDet;
        result[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invDet;
        result[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * invDet;
        result[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * invDet;
        result[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * invDet;
        result[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * invDet;

        return true;
    }

    // 係数を計算する
    void calculateCoefficients() {
        float X[3][3] = {0};
        float Y[3][1] = {0};

        for (uint8_t i = 0; i < 3; ++i) {
            X[0][0] += 1;
            X[0][1] += sampleX[i];
            X[0][2] += sampleX[i] * sampleX[i];
            X[1][0] += sampleX[i];
            X[1][1] += sampleX[i] * sampleX[i];
            X[1][2] += sampleX[i] * sampleX[i] * sampleX[i];
            X[2][0] += sampleX[i] * sampleX[i];
            X[2][1] += sampleX[i] * sampleX[i] * sampleX[i];
            X[2][2] += sampleX[i] * sampleX[i] * sampleX[i] * sampleX[i];
            Y[0][0] += sampleY[i];
            Y[1][0] += sampleY[i] * sampleX[i];
            Y[2][0] += sampleY[i] * sampleX[i] * sampleX[i];
        }

        float invX[3][3];
        if (invertMatrix(X, invX)) {
            multiplyMatrices(invX, Y, coefficients);
        } else {
            for (uint8_t i = 0; i < 3; ++i) {
                coefficients[i][0] = 0;
            }
        }
    }
};

#endif