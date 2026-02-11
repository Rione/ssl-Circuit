#include "robot.hpp"

Robot::Robot() {
}

void Robot::Initialize() {
  led0 = 1;
  printf("Robot Initialize Start\n");
  HAL_Delay(100);

  // uartの初期化
  serial1.init();
  serial4.init();
  for (auto& serial : md_serials) {
    serial.init();
  }

  // CANの初期化
  can.init();

  printf("Robot Initialize Finish\n");
  led0 = 0;
}

void Robot::RockRecvSerial(RobotInfo& info) {
  const uint8_t kHeader = 0xFF;
  const uint8_t kFooter = 0xAA;
  const uint8_t kDataSize = 18;  // 受信データサイズ(バイト数)
  static uint8_t recv_data[kDataSize];
  static uint8_t index = 0;
  uint8_t recv_byte;

  if (serial1.available()) {
    recv_byte = serial1.getc();

    if (index == 0) {
      if (recv_byte == kHeader) {  // ヘッダー確認
        index++;
      } else {
        index = 0;
      }
    } else if (index == (kDataSize + 1)) {
      if (recv_byte == kFooter) {  // フッター確認
        // 正常にデータを受信
        info.vel_x.l = recv_data[0];
        info.vel_x.h = recv_data[1];
        info.vel_y.l = recv_data[2];
        info.vel_y.h = recv_data[3];
        info.vel_angular.l = recv_data[4];
        info.vel_angular.h = recv_data[5];
        info.dribble_power = recv_data[6];
        info.kicker.straight = recv_data[7];
        info.kicker.chip = recv_data[8];

        info.relative_position_x.l = recv_data[9];
        info.relative_position_x.h = recv_data[10];
        info.relative_position_y.l = recv_data[11];
        info.relative_position_y.h = recv_data[12];
        info.relative_theta.l = recv_data[13];
        info.relative_theta.h = recv_data[14];
        info.camera.x = recv_data[15];
        info.camera.y = recv_data[16];

        info.status.data = recv_data[17];
      }
      index = 0;
    } else {
      recv_data[index - 1] = recv_byte;
      index++;
    }
  }
}

void Robot::RockSendSerial(RobotInfo& info) {
  const uint8_t kDataSize = 3;  // 送信データサイズ(バイト数)
  const uint8_t kFooter[4] = {0xFF, 0, 0xFF, 0};
}