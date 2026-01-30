#include "robot.hpp"

Robot::Robot() {
}

void Robot::Initialize() {
      printf("Robot Initialize Start\n");
      serial1.init();
      serial2.init();
      serial3.init();
      serial4.init();
      serial5.init();
      serial6.init();

      can.init();

      printf("Robot Initialize Begin\n");
}

void Robot::RockRecvSerial(RobotInfo_t& info) {
}

void Robot::RockSendSerial(RobotInfo_t& info) {
}

void Robot::UiRecvSerial(RobotInfo_t& info) {
}

void Robot::UiSendSerial(RobotInfo_t& info) {
}