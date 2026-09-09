#ifndef IMU_TEST_H_
#define IMU_TEST_H_

// LSM6DSO32XTR (SPI1) の動作確認用テストプログラム
// USART1(printf)へ加速度・ジャイロ・温度を継続表示する
void ImuTest_Run(void);

#endif  // IMU_TEST_H_
