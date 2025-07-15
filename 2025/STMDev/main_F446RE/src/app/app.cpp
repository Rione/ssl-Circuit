#include "app.h"

#include "RobotSequence.hpp"
#include "cameraMode.hpp"
#include "mainMode.hpp"
#include "testMode.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);
TestMode testMode('T', "Test Mode", &robot);
CameraMode cameraMode('C', "Camera Mode", &robot);

void TimInterrupt1khz() {
      robot.heartBeat();
      robot.swImu.update();
      robot.swDischarge.update();
}

void TimInterrupt4khz() {
      // robot.mpuget();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
      if (robot.can.getHcan() == hcan) {
            robot.can.recv(canRecvData);
            switch (canRecvData.stdId) {
                  // KickerBoard Commands
                  case CHARGE_START:  // 16: charge Enable
                        robot.led2 = true;
                        break;
                  case DISCHARGE_START:  // 17: charge Disable
                        robot.kickerBoard.setCapValEstimate(0);
                        robot.led2 = false;
                        break;
                  case KICK_STRAIGHT:  // 18: kick
                        robot.kickerBoard.minusCapValEstimate(canRecvData.data[0]);
                        break;
                  case KICK_CHIP:  // 19: chip kick
                        robot.kickerBoard.minusCapValEstimate(canRecvData.data[0]);
                        break;
                  case KICKER_BOARD_PACKET:  // フォトセンサの値・充電完了信号の受信
                        // robot.setPhotoSensorValue((uint16_t)(canRecvData.data[0]) | (uint16_t)(canRecvData.data[1]) << 8); // フォトセンサの値の処理はドリブラで行う
                        robot.setChageDoneSignal(canRecvData.data[2]);                      // 充電完了信号の処理
                        robot.setKickerBoardChargeMode(canRecvData.data[3]);                // 充電モード信号の処理
                        robot.info.kickerBoardDoDirectStatus.status = canRecvData.data[4];  // doDirectの状態を受信
                        // uint16_t photoSensorThreshold = (uint16_t)(canRecvData.data[5]) | (uint16_t)(canRecvData.data[6]) << 8;
                        robot.led0 = !robot.led0;
                        break;
                  case DRIBBLE_RECV:
                        robot.info.dribbleStatus.isHoldBall = (canRecvData.data[0] != 0);
                        // data[1],data[2,3]ドリブラ検知の値は無視　CAN監視用
                        robot.info.dribbleStatus.isDetectedBall = (canRecvData.data[4] != 0);
                        robot.info.photoSensorValue = (uint16_t)(canRecvData.data[5]) | (uint16_t)(canRecvData.data[6]) << 8;
                        break;
                  case MD0_RECV:
                        robot.info.mdStatus.motorAngularVelocity[0] = (uint8_t)(canRecvData.data[0]) | (uint8_t)(canRecvData.data[1]) << 8;
                        break;
                  case MD1_RECV:
                        robot.info.mdStatus.motorAngularVelocity[1] = (uint8_t)(canRecvData.data[0]) | (uint8_t)(canRecvData.data[1]) << 8;
                        break;
                  case MD2_RECV:
                        robot.info.mdStatus.motorAngularVelocity[2] = (uint8_t)(canRecvData.data[0]) | (uint8_t)(canRecvData.data[1]) << 8;
                        break;
                  case MD3_RECV:
                        robot.info.mdStatus.motorAngularVelocity[3] = (uint8_t)(canRecvData.data[0]) | (uint8_t)(canRecvData.data[1]) << 8;
                        break;
                  default:
                        break;
            }
      }
}

void setup() {
      robot.hardwareInit();
      HAL_Delay(100);
      if (robot.swImu.read() == 1) {
            robot.info.runMode = 0;
      } else {
            robot.info.runMode = 1;
      }
}

void main_app() {
      while (1) {
            if (robot.info.runMode == 0) {
                  mainMode.loop();
            } else {
                  cameraMode.loop();
            }
      }
}