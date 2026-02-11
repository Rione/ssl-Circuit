#include "app.h"

#include "main_mode.hpp"
#include "robot.hpp"

Robot robot;
CANBus::CANData canData;
MainMode mainMode(&robot);

void Setup() {
      robot.Initialize();
}

void MainApp() {
      while (1) {
            mainMode.Loop();
      }
}