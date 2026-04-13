#include "motor_drive.hpp"

MotorDrive::MotorDrive(BufferedSerial* serial)
    : _emg(false), _ready(false), _serial(serial) {
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

            // 結果を格納
            m[i] = (int16_t)v_wheel_angular;
      }

      Send(m);
}

void MotorDrive::Send(int16_t* m) {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      uint8_t send_data[10];

      send_data[0] = HEADER;
      send_data[1] = (m[0] >> 8) & 0xFF;
      send_data[2] = m[0] & 0xFF;
      send_data[3] = (m[1] >> 8) & 0xFF;
      send_data[4] = m[1] & 0xFF;
      send_data[5] = (m[2] >> 8) & 0xFF;
      send_data[6] = m[2] & 0xFF;
      send_data[7] = (m[3] >> 8) & 0xFF;
      send_data[8] = m[3] & 0xFF;
      send_data[9] = FOOTER;

      _serial->write(send_data, 10);
}

void MotorDrive::Recv() {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 3;  // 受信データサイズ(バイト数)
      static uint8_t recv_data[3];
      static uint8_t index = 0;
      uint8_t recv_byte;

      if (_serial->available()) {
            recv_byte = _serial->getc();

            if (index == 0) {
                  if (recv_byte == HEADER) {  // ヘッダー確認
                        index++;
                  } else {
                        index = 0;
                  }
            } else if (index == (data_size + 1)) {
                  if (recv_byte == FOOTER) {  // フッター確認
                        // データ受信完了
                        _emg = recv_data[0] & 0b00000001;                                          // 非常停止信号
                        _ready = (recv_data[0] >> 1) & 0b00000001;                                 // ドライバ準備完了信号
                        _vel_wheel_angular[0] = (int16_t)((recv_data[1] << 8) | recv_data[2]);       // 角速度[rad/s]
                  }
                  index = 0;
            } else {
                  recv_data[index - 1] = recv_byte;
                  index++;
            }
      }
}

void MotorDrive::GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
      // 各ホイールの角速度[rad/s]から線速度[m/s]に変換
      float v_wheel_linear[4];
      for (int i = 0; i < 4; i++) {
            v_wheel_linear[i] = _vel_wheel_angular[i] * params::WHEEL_RADIUS;
      }

      // 順運動学：各成分の和を計算
      float vx_sum = 0.0f;
      float vy_sum = 0.0f;
      float v_sum = 0.0f;

      for (int i = 0; i < 4; i++) {
            vx_sum += v_wheel_linear[i] * (-MyMath::sinDeg(params::MOTOR_DEGREE[i]));
            vy_sum += v_wheel_linear[i] * MyMath::cosDeg(params::MOTOR_DEGREE[i]);
            v_sum += v_wheel_linear[i];
      }

      // ロボットの速度を計算
      float vx_m = vx_sum / 2.0f;
      float vy_m = vy_sum / 2.0f;
      float omega = v_sum / (4.0f * params::WHEEL_BASE_RADIUS);

      // [m/s] -> [mm/s] に変換
      *vel_x = (int16_t)(vx_m * 1000.0f);
      *vel_y = (int16_t)(vy_m * 1000.0f);
      *vel_angle = (int16_t)omega;
}