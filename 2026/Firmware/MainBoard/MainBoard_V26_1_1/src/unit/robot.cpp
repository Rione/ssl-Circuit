#include "robot.hpp"

Robot::Robot() {
}

void Robot::Initialize() {
      led0 = 1;
      printf("Robot Initialize Start\n");
      HAL_Delay(100);

      serial1.init();
      serial4.init();
      for (auto& serial : mdSerials) serial.init();

      can.init();

      printf("Robot Initialize Finish\n");
      led0 = 0;
}

void Robot::RockRecvSerial(RobotInfo_t& info) {
}

void Robot::RockSendSerial(RobotInfo_t& info) {
}

void Robot::UiRecvSerial(RobotInfo_t& info) {
}

void Robot::UiSendSerial(RobotInfo_t& info) {
}