#ifndef __WHEEL_VOLTAGE_H_
#define __WHEEL_VOLTAGE_H_

// 電圧制御 (疑似トルク制御) のフィードフォワードのうち、速度を保つ分:
//   V = f(輪,向き, ω*) + Vd(輪)·s(ω*)   (Vd は床の負荷分、WHEEL_VOLT_USE_LOAD_FF で有効)
// ω* [rad/s] は各輪の目標角速度。f は車輪を浮かせて一定電圧をかけたときの定常回転数の表
// (引き継ぎ文書 5.3 の測り直し) を輪ごと・向きごとに逆に引いたもの。WheelUnit のキャリブレーションを
// やり直したら測り直して更新すること。加減速ぶんは omni_drive.c で機体の加速度から力の配分で足す。

// 輪 wheel (0-3) を角速度 omega_ref [rad/s] で回し続けるのに要る電圧 [V] を返す (±WHEEL_VOLT_MAX でクランプ済み)
float WheelVoltage_Feedforward(int wheel, float omega_ref);

#endif  // __WHEEL_VOLTAGE_H_
