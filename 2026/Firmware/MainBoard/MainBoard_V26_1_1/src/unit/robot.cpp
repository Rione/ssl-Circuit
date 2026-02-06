#include "robot.hpp"

Robot::Robot() {
}

void Robot::Initialize() {
      printf("Robot Initialize Start\n");
      serial1.init();
      serial4.init();
      for (auto& serial : mdSerials) serial.init();

      can.init();

      printf("Robot Initialize Finish\n");
}

void Robot::RockRecvSerial(RobotInfo_t& info) {
}

void Robot::RockSendSerial(RobotInfo_t& info) {
}

void Robot::UiRecvSerial(RobotInfo_t& info) {
}

void Robot::UiSendSerial(RobotInfo_t& info) {
}

void Robot::MDSendSerial(RobotInfo_t& info) {
}

void Robot::MDRecvSerial(RobotInfo_t& info) {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;
      const uint8_t data_size = 2;
      static uint8_t recv_data[2];
      static uint8_t index = 0;
      uint8_t recv_byte = Serial_Read(&uart2);

      if (Serial_Available(&uart2)) {
            enable = true;
            if (index == 0) {
                  if (recv_byte == HEADER) {
                        index++;
                  } else {
                        index = 0;
                  }
            } else if (index == 1) {
                  if (recv_byte == SPEED_HEADER) {
                        mode = 1;  // 速度制御モード
                        index++;
                  } else if (recv_byte == POSITION_HEADER) {
                        mode = 2;  // 位置制御モード
                        index++;
                  } else if (recv_byte == TORQUE_HEADER) {
                        mode = 3;  // トルク制御モード
                        index++;
                  } else {
                        index = 0;
                  }
            } else if (index == (data_size + 2)) {
                  if (recv_byte == FOOTER) {
                        PwmOut_Write(&LED3, 1);
                        if (mode == 1) {
                              target_speed = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.01;  // 速度制御
                        } else if (mode == 2) {
                              target_position = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.001;  // 位置制御
                        } else if (mode == 3) {
                              target_torque = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.001;  // トルク制御
                        }
                  }
                  index = 0;
            } else {
                  recv_data[index - 2] = recv_byte;
                  index++;
            }
            Timer_Reset(&serial_recv_timer);
      } else if (Timer_Read(&serial_recv_timer) > 1) {  // 100msごとにシリアル受信
            enable = false;
            PwmOut_Write(&LED3, 0);
            Serial_Reset(&uart2);
            Timer_Reset(&serial_recv_timer);
      }
}