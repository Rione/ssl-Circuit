#include "RobotSequence.hpp"

extern Robot robot;

void boosterManager() {
  static Timer uiBoosterCheckInterval;
  static Timer doChargeTimer;
  static Timer manageByUserCounter;
  if (manageByUserCounter.read_ms() > 15000) manageByUserCounter.set_ms(15000);
  if (uiBoosterCheckInterval.read_ms() > 1000) uiBoosterCheckInterval.set_ms(1000);

  if (uiBoosterCheckInterval.read_ms() > 100) {
    uiBoosterCheckInterval.reset();

    robot.UIrobotInfo.batteryGet = robot.info.batteryVoltage;
    robot.UIrobotInfo.capaData.chargeState = robot.info.isKickerChargeMode;
    robot.UIrobotInfo.capaData.chargeVol = robot.info.capChargeCertitude;

    robot.serial4.write(0xFF);
    robot.serial4.write(robot.UIrobotInfo.batteryGet);
    robot.serial4.write(robot.UIrobotInfo.capaData.data);
    robot.serial4.write((uint8_t)robot.UIrobotInfo.buzzer);

    if (robot.serial4.available()) {
      static const uint8_t HEADER = 0xFF;
      static const uint8_t dataSize = 1;
      static bool headerReceived = false;
      static uint8_t index = 0;
      static uint8_t data[dataSize] = {0};

      while (robot.serial4.available()) {
        uint8_t receivedByte = robot.serial4.read();
        if (!headerReceived) {
          index = 0;
          if (receivedByte == HEADER) headerReceived = true;
        } else {
          data[index] = receivedByte;
          index++;
          if (index == dataSize) {
            robot.UImodeData.status.data = data[0];
            headerReceived = false;
            index = 0;
          }
        }
      }

      if (robot.UImodeData.status.charge_state == 1) {
        if (robot.info.isKickerChargeMode == false) {
          robot.chargeStart();
          robot.led2 = true;
        } else {
          robot.discharge();
          robot.led2 = false;
        }

        robot.UImodeData.status.charge_state = 0;

      } else if (robot.UImodeData.status.kick == 1) {
        robot.kickStraight(50);
        robot.UImodeData.status.kick = 0;
      }

      manageByUserCounter.reset();
    }
  }

  if (robot.swDischarge.isRelease()) {
    if (robot.swDischarge.readPressedTime() > 1600) {
      robot.discharge();
      robot.led2 = false;
    } else if (robot.swDischarge.readPressedTime() > 800) {
      robot.chargeStart();
      robot.led2 = true;
    } else if (robot.swDischarge.readPressedTime() > 200) {
      robot.kickStraight(100);
    }
    manageByUserCounter.reset();
  } else {
    if (manageByUserCounter.read_ms() >= 15000) {
      if (doChargeTimer.read_ms() > 100) {
        if (robot.info.status.doCharge == true) {
          static uint8_t countD = 0;
          if (robot.info.isKickerChargeMode == false) {
            countD++;
            if (countD > 10) {
              robot.chargeStart();
              countD = 0;
            }
          }
        } else {
          static uint8_t countC = 0;
          if (robot.info.isKickerChargeMode == true) {
            countC++;
            if (countC > 10) {
              robot.discharge();
              countC = 0;
            }
          }
        }
        doChargeTimer.reset();
      }
    }
  }
}