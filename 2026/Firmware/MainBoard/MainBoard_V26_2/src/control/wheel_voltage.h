#ifndef __WHEEL_VOLTAGE_H_
#define __WHEEL_VOLTAGE_H_

// 電圧制御 (疑似トルク制御) のフィードフォワード:
//   V = f(輪,向き, ω*) + Ka·α*
// ω* [rad/s], α* [rad/s^2] は各輪の目標角速度・目標角加速度。
// f は車輪を浮かせて一定電圧をかけたときの定常回転数の表 (引き継ぎ文書 5.3 の測り直し) を
// 輪ごと・向きごとに逆に引いたもの。WheelUnit のキャリブレーションをやり直したら測り直して更新すること。

// 輪 wheel (0-3) に印加する電圧 [V] を返す (±WHEEL_VOLT_MAX でクランプ済み)
float WheelVoltage_Feedforward(int wheel, float omega_ref, float alpha_ref);

#endif  // __WHEEL_VOLTAGE_H_
