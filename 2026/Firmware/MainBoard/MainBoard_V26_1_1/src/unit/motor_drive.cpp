#include "motor_drive.hpp"

MotorDrive::MotorDrive(BufferedSerial* serials[4]) {
      for (int i = 0; i < 4; i++) _serials[i] = serials[i];
}

void MotorDrive::SetVel(int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
      // vel_x, vel_y [mm/s] -> [m/s]
      float vx_m = vel_x / 1000.0f;
      float vy_m = vel_y / 1000.0f;

      int16_t m[4];

      for (int i = 0; i < 4; i++) {
            // 各ホイールの接線方向の速度 [m/s] を計算
            // オムニホイールの逆運動学: v_w = -vx * sin(theta) + vy * cos(theta) + R * omega
            float v_wheel_linear = -vx_m * MyMath::sinDeg(params::MOTOR_DEGREE[i]) +
                                   vy_m * MyMath::cosDeg(params::MOTOR_DEGREE[i]) +
                                   params::WHEEL_BASE_RADIUS * vel_angle;

            // タイヤの角速度[rad/s] に変換
            // omega = v / r
            float v_wheel_angular = v_wheel_linear / params::WHEEL_RADIUS;

            // 結果を格納 (int16_tの範囲に収まるか注意が必要ですが、ここではそのままキャスト)
            m[i] = (int16_t)v_wheel_angular;
      }

      SendMotors(m[0], m[1], m[2], m[3]);
}

void MotorDrive::SendMotors(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      uint8_t send_data[10];

      send_data[0] = HEADER;

      send_data[9] = FOOTER;
}

void MotorDrive::SendEmg() {
}

void MotorDrive::GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
}