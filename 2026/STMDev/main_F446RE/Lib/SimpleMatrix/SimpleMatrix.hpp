// ファイル名: SimpleMatrix.hpp
#ifndef SIMPLE_MATRIX_HPP
#define SIMPLE_MATRIX_HPP

#include <cmath>
#include <cstdint> // uint32_tなどのために必要
#include <cstring>

class SimpleMatrix {
public:
  int rows;
  int cols;
  float *data;

  SimpleMatrix(int r, int c);
  SimpleMatrix(const SimpleMatrix &other); // コピーコンストラクタ
  ~SimpleMatrix();                         // デストラクタ

  SimpleMatrix &operator=(const SimpleMatrix &other); // 代入演算子

  // アクセス
  float &at(int r, int c);
  float get(int r, int c) const;
  void set(int r, int c, float val);

  // 演算
  SimpleMatrix operator+(const SimpleMatrix &other) const;
  SimpleMatrix operator-(const SimpleMatrix &other) const;
  SimpleMatrix operator*(const SimpleMatrix &other) const;
  SimpleMatrix operator*(float scalar) const;

  // 転置
  SimpleMatrix transpose() const;

  // 逆行列 (2x2専用)
  SimpleMatrix inverse2x2() const;

  // 単位行列
  static SimpleMatrix identity(int size);

  // ゼロ埋め
  void setZero();
};

#endif
