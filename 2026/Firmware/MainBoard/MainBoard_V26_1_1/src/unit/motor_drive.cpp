#include "motor_drive.hpp"

MotorDrive::MotorDrive(BufferedSerial* serials) {
      for (int i = 0; i < 4; i++) serials_[i] = &serials[i];
}

void MotorDrive::SetVel(int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
      // vel_x, vel_y [mm/s] -> [m/s]
      float vx_m = vel_x / 1000.0f;
      float vy_m = vel_y / 1000.0f;

      int16_t m[4];

      for (int i = 0; i < 4; i++) {
            // 各ホイールの接線方向の速度 [m/s] を計算
            // オムニホイールの逆運動学: v_w = -vx * sin(theta) + vy * cos(theta) + R * omega
            float v_wheel_linear = -vx_m * MyMath::sinDeg(robot_params::kMotorDegree[i]) +
                                   vy_m * MyMath::cosDeg(robot_params::kMotorDegree[i]) +
                                   robot_params::kWheelBaseRadius * vel_angle;

            // タイヤの角速度[rad/s] に変換
            // omega = v / r
            float v_wheel_angular = v_wheel_linear / robot_params::kWheelRadius;

            // 結果を格納
            m[i] = (int16_t)v_wheel_angular;
      }

      Send(m);
}

void MotorDrive::Send(int16_t* m) {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 10;  // 送信データサイズ(バイト数)
      uint8_t send_data[data_size];

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

      serials_[0]->write(send_data, data_size);
}

void MotorDrive::Recv() {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 3;  // 受信データサイズ(バイト数)
      static uint8_t recv_data[4][data_size];
      static uint8_t index[4] = {0};
      uint8_t recv_byte[4];

      for (int i = 0; i < 4; i++) {
            if (serials_[i]->available()) {
                  recv_byte[i] = serials_[i]->getc();

                  if (index[i] == 0) {
                        if (recv_byte[i] == HEADER) {  // ヘッダー確認
                              index[i]++;
                        } else {
                              index[i] = 0;
                        }
                  } else if (index[i] == (data_size + 1)) {
                        if (recv_byte[i] == FOOTER) {  // フッター確認
                              // 正常にデータを受信
                              emg_ = recv_data[i][0] & 0b00000001;                                          // 非常停止信号
                              ready_ = (recv_data[i][0] >> 1) & 0b00000001;                                 // ドライバ準備完了信号
                              vel_wheel_angular_[i] = (int16_t)((recv_data[i][1] << 8) | recv_data[i][2]);  // 角速度[rad/s]
                        }
                        index[i] = 0;
                  } else {
                        recv_data[i][index[i] - 1] = recv_byte[i];
                        index[i]++;
                  }
            }
      }
}

void MotorDrive::GetVel(int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
      // 各ホイールの角速度[rad/s]から線速度[m/s]に変換
      float v_wheel_linear[4];
      for (int i = 0; i < 4; i++) {
            v_wheel_linear[i] = vel_wheel_angular_[i] * robot_params::kWheelRadius;
      }

      // 順運動学：各成分の和を計算
      float vx_sum = 0.0f;
      float vy_sum = 0.0f;
      float v_sum = 0.0f;

      for (int i = 0; i < 4; i++) {
            vx_sum += v_wheel_linear[i] * (-MyMath::sinDeg(robot_params::kMotorDegree[i]));
            vy_sum += v_wheel_linear[i] * MyMath::cosDeg(robot_params::kMotorDegree[i]);
            v_sum += v_wheel_linear[i];
      }

      // ロボットの速度を計算
      float vx_m = vx_sum / 2.0f;
      float vy_m = vy_sum / 2.0f;
      float omega = v_sum / (4.0f * robot_params::kWheelBaseRadius);

      // [m/s] -> [mm/s] に変換
      *vel_x = (int16_t)(vx_m * 1000.0f);
      *vel_y = (int16_t)(vy_m * 1000.0f);
      *vel_angle = (int16_t)omega;
}