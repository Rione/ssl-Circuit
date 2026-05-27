// ファイル名: SimpleMatrix.cpp
#include "SimpleMatrix.hpp"
#include <algorithm>
#include <iostream>

SimpleMatrix::SimpleMatrix(int r, int c) : rows(r), cols(c) {
  data = new float[rows * cols];
  // 初期化なし(高速化のため) or setZero呼び出し
  setZero();
}

SimpleMatrix::SimpleMatrix(const SimpleMatrix &other)
    : rows(other.rows), cols(other.cols) {
  data = new float[rows * cols];
  std::memcpy(data, other.data, sizeof(float) * rows * cols);
}

SimpleMatrix::~SimpleMatrix() { delete[] data; }

SimpleMatrix &SimpleMatrix::operator=(const SimpleMatrix &other) {
  if (this == &other)
    return *this;
  delete[] data;
  rows = other.rows;
  cols = other.cols;
  data = new float[rows * cols];
  std::memcpy(data, other.data, sizeof(float) * rows * cols);
  return *this;
}

float &SimpleMatrix::at(int r, int c) { return data[r * cols + c]; }

float SimpleMatrix::get(int r, int c) const { return data[r * cols + c]; }

void SimpleMatrix::set(int r, int c, float val) { 
  data[r * cols + c] = val; 
  //printf("%d",cols);
  //printf("%f",val);
}

SimpleMatrix SimpleMatrix::operator+(const SimpleMatrix &other) const {
  SimpleMatrix res(rows, cols);
  for (int i = 0; i < rows * cols; i++) {
    res.data[i] = data[i] + other.data[i];
  }
  return res;
}

SimpleMatrix SimpleMatrix::operator-(const SimpleMatrix &other) const {
  SimpleMatrix res(rows, cols);
  for (int i = 0; i < rows * cols; i++) {
    res.data[i] = data[i] - other.data[i];
  }
  return res;
}

SimpleMatrix SimpleMatrix::operator*(const SimpleMatrix &other) const {
  // A(m x n) * B(n x p) = C(m x p)
  SimpleMatrix res(rows, other.cols);
  res.setZero();
  for (int i = 0; i < rows; i++) {
    for (int k = 0; k < cols; k++) {
      float valA = get(i, k);
      for (int j = 0; j < other.cols; j++) {
        res.data[i * other.cols + j] += valA * other.data[k * other.cols + j];
      }
    }
  }
  return res;
}

SimpleMatrix SimpleMatrix::operator*(float scalar) const {
  SimpleMatrix res(rows, cols);
  for (int i = 0; i < rows * cols; i++) {
    res.data[i] = data[i] * scalar;
  }
  return res;
}

SimpleMatrix SimpleMatrix::transpose() const {
  SimpleMatrix res(cols, rows);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      res.at(j, i) = get(i, j);
    }
  }
  return res;
}

// 2x2専用逆行列 (簡易実装)
SimpleMatrix SimpleMatrix::inverse2x2() const {
  SimpleMatrix res(2, 2);
  float det = get(0, 0) * get(1, 1) - get(0, 1) * get(1, 0);
  if (std::abs(det) < 1e-9f) {
    // 特異行列
    res.setZero();
    return res;
  }
  float invDeb = 1.0f / det;
  res.set(0, 0, get(1, 1) * invDeb);
  res.set(0, 1, -get(0, 1) * invDeb);
  res.set(1, 0, -get(1, 0) * invDeb);
  res.set(1, 1, get(0, 0) * invDeb);
  return res;
}

SimpleMatrix SimpleMatrix::identity(int size) { //単位行列生成
  SimpleMatrix res(size, size);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      if (i == j)
        res.set(i, j, 1.0f);
      else
        res.set(i, j, 0.0f);
    }
  }
  return res;
}

void SimpleMatrix::setZero() {
  std::memset(data, 0, sizeof(float) * rows * cols);
}
