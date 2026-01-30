#include "app.hpp"

#include "robot.hpp"

Robot robot;
CANBus::CANData canData;

void Setup() {
      robot.Initialize();
      HAL_Delay(1000);
}

void MainApp() {
      while (1) {
            MainApp();
      }
}