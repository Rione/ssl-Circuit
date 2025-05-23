#include "Robot.hpp"

Robot::Robot() {
}

void Robot::hardwareInit() {
      // led1 = bno.check();
      // bno.setUnit(1, 1, 1, 0);
      // bno.setPowerMode();
      // // bno.setOperaitonMode(OPERATION_MODE_AMG);
      // bno.setOperaitonMode(OPERATION_MODE_NDOF);
      // // bno.accConfig();
      // bno.init();
      can.init();

      serial1.init();
      serial4.init();
      serial5.init();

      ledH.init();
      printf("Hello World\n");

      medianPhotoValue.init();
      medianBatteryValue.init();

      filter.begin(33000);  // 4000のはずなんだけど、何故か8.25倍しないとmain_appで使い物にならなかった...

      mpu.init();

      // bno.setAttitudeZero();
      // HAL_Delay(1000);
}

void Robot::rasRecvSerial(RobotInfo_t &info) {
      static const uint8_t HEADER = 0xFF;   // ヘッダ
      static const uint8_t dataSize = 18;   // データのサイズ
      static bool headerReceived = false;   // ヘッダを受信したかどうか
      static uint8_t index = 0;             // 受信したデータのインデックスカウンター
      static uint8_t data[dataSize] = {0};  // 受信したデータ

      while (serial5.available()) {
            // 1バイト読み込み
            uint8_t receivedByte = serial5.read();
            // printf("received %d\n ", receivedByte);

            if (!headerReceived) {
                  index = 0;
                  if (receivedByte == HEADER) {
                        // ヘッダを受信したらデータの受信を開始
                        headerReceived = true;  // ヘッダを受信したフラグを立てる
                                                // printf("header received %d\n ", receivedByte);
                  } else {
                        headerReceived = false;  // ヘッダではないのでフラグをリセット
                                                 // printf("error: Header is not received %d\n", receivedByte);
                  }
            } else {  // ヘッダを受信した後の処理
                  if (index < dataSize) {
                        // データ受信
                        data[index] = receivedByte;
                        // printf("data[%d]: %d\n", index, data[index]);
                        index++;

                        if (index == dataSize) {
                              // データ受信完了
                              info.velX.L = data[0];
                              info.velX.H = data[1];
                              info.velY.L = data[2];
                              info.velY.H = data[3];
                              info.velAngler.L = data[4];
                              info.velAngler.H = data[5];
                              info.dribblePower = data[6];
                              info.kicker.straight = data[7];
                              info.kicker.chip = data[8];

                              info.relativePositionX.L = data[9];
                              info.relativePositionX.H = data[10];
                              info.relativePositionY.L = data[11];
                              info.relativePositionY.H = data[12];
                              info.relativeTheta.L = data[13];
                              info.relativeTheta.H = data[14];
                              info.camera.x = data[15];
                              info.camera.y = data[16];

                              info.status.data = data[17];

                              headerReceived = false;  // 次のヘッダを待つ準備をする
                              index = 0;               // インデックスをリセット
                        }
                  }
            }
      }
}

void Robot::rasSendSerial(RobotInfo_t &info, uint16_t interval) {
      static const uint8_t dataSize = 3;  // データのサイズ
      static const uint8_t startBytes[4] = {0xFF, 0, 0xFF, 0};
      static Timer timer;

      if (timer.read_ms() < interval) {
            return;
      }

      info.dribbleStatus.isNewDrib = true;  // 新機体

      uint8_t buffer[dataSize] = {
          info.batteryVoltage,
          info.dribbleStatus.data,
          info.capValEstimate,
      };
      serial5.write(startBytes, 4);
      serial5.write(buffer, dataSize);
      timer.reset();
}

void Robot::getSensors(RobotInfo_t *info) {
      static uint16_t underVoltageCount = 0;
      // バッテリー電圧
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 100);
      uint32_t batt = HAL_ADC_GetValue(&hadc1);
      info->batteryVoltage = medianBatteryValue.calc((uint8_t)((float)(batt) * 3.3 / 4095.0 * 58.5 - 5));
      if (info->batteryVoltage < BATTERY_CUT_OFF) {
            if (underVoltageCount < 1000) underVoltageCount++;
      } else {
            if (underVoltageCount > 0) underVoltageCount--;
      }
      info->isUnderVoltage = (underVoltageCount == 0);
      info->capValEstimate = kickerBoard.getCapValEstimate();
}

void Robot::sendDribble(uint8_t power, bool forceSend) {
      static Timer timer;
      static uint8_t dribblePowerPrev = power;
      if (timer.read_ms() > 10000) timer.set_ms(10000);
      if (power == dribblePowerPrev && forceSend == false) {
            if (timer.read_ms() < 100)  // パワーが変わっていない場合は送信しない。 forceSendがtrueの場合は100msごとに送信する
                  return;
      }
      power = (power != 0) ? 100 : 0;
      CANBus::CANData canData = {
          .stdId = DRIBBLE_SEND,
          .data = {power, 0, 0, 0, 0, 0, 0, 0},
      };
      can.send(canData);
      timer.reset();
      dribblePowerPrev = power;
}

void Robot::sendKicker(RobotInfo_t &info) {
      // キックの処理
      // ストレートを優先してキック
      if (info.kicker.straight > 0) {
            kickerBoard.kick(STRAIGHT, info.kicker.straight, info.status.doDirectKick);
      } else if (info.kicker.chip > 0) {
            kickerBoard.kick(CHIP, info.kicker.chip, info.status.doDirectChipKick);
      } else {
            // どっちも0の場合はキックしない
            if (info.status.doDirectKick != info.kickerBoardDoDirectStatus.straight && info.kickerBoardDoDirectStatus.straight) {
                  // kickStraight(0, false); // パワー0のキックを投げてdoDirectをリセットする
                  kickerBoard.resetDoDirect(STRAIGHT);
            }
            if (info.status.doDirectChipKick != info.kickerBoardDoDirectStatus.chip && info.kickerBoardDoDirectStatus.chip) {
                  // kickChip(0, false); // パワー0のキックを投げてdoDirectをリセットする
                  kickerBoard.resetDoDirect(CHIP);
            }
      }
}

void Robot::sendMotor(RobotInfo_t &info, uint8_t interval) {
      // MDの処理
      // モータードライバの送信速度決め
      int16_t __velX = meanVelX.calc((float)info.velX.vel);
      int16_t __velY = meanVelY.calc((float)info.velY.vel);
      int16_t __velAngler = meanVelAngler.calc((float)info.velAngler.vel);
      static uint8_t md_sendcount = 0;
      md_sendcount++;
      if (md_sendcount % interval == 0) {  // 5msごとに送信
            motorDriver.setVelocityFF(__velX, __velY, __velAngler);
      }
}

// FF速度ベクトルからロボットが止まっているかを判断する
void Robot::checkRobotRest(RobotInfo_t &info) {
      static int velZeroCount = 0;
      if (info.velX.vel == 0 && info.velY.vel == 0 && info.velAngler.vel == 0) {
            if (velZeroCount < 1000) {
                  velZeroCount++;
            } else {
                  velZeroCount = 1000;
                  info.isRobotStopFF = true;
            }
      } else {
            velZeroCount = 0;
            info.isRobotStopFF = false;
      }
}

void Robot::uiSendSerial(RobotInfo_t &info, uint16_t interval) {
      // UIにデータを送信する
      static const uint8_t HEADER = 0xFF;  // ヘッダ
      static const uint8_t dataSize = 3;   // データのサイズ
      static Timer timer;

      if (timer.read_ms() < interval) {
            return;
      }

      info.capaData.chargeState = info.isKickerChargeMode;
      info.capaData.chargeVal = info.capValEstimate;

      uint8_t buffer[dataSize] = {
          info.batteryVoltage,
          info.capaData.data,
          (uint8_t)info.buzzer,
      };
      serial4.write(HEADER);
      serial4.write(buffer, dataSize);
      printf("send %d %d %d\n", buffer[0], info.capaData.chargeState, info.capaData.chargeVal);

      timer.reset();
}

void Robot::uiRecvSerial(RobotInfo_t &info) {
      // UIにデータを送信する
      static const uint8_t HEADER = 0xFF;   // ヘッダ
      static const uint8_t dataSize = 1;    // データのサイズ
      static bool headerReceived = false;   // ヘッダを受信したかどうか
      static uint8_t index = 0;             // 受信したデータのインデックスカウンター
      static uint8_t data[dataSize] = {0};  // 受信したデータ

      while (serial4.available()) {
            uint8_t receivedByte = serial4.read();
            if (!headerReceived) {
                  index = 0;
                  if (receivedByte == HEADER) headerReceived = true;
            } else {
                  data[index] = receivedByte;
                  index++;
                  if (index == dataSize) {
                        info.uiStatus.data = data[0];
                        headerReceived = false;
                        index = 0;
                  }
            }
      }
}

void Robot::mpuget(RobotInfo_t &info) {
      if (mpu.isCalibrated() == true) {
            mpu.getAccGyro(&acc, &gyro, false);
            filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
            att.z = (float)MyMath::gapDegrees180(filter.getYaw(), info.frontDeg);
      }
}

void Robot::mpuSetup(RobotInfo_t &info) {
      if (swImu.read() == false) {
            // set flash
            printf("IMU calibrating\n");
            HAL_Delay(1000);
            mpu.calibrateAccGyro();
            mpu.getOffset(&info.imuOffsets.acc, &info.imuOffsets.gyro);
            flash.writeFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            HAL_Delay(1000);
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            printf("ACC offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
      } else {
            // load flash オフセット値をFlashから読み出す(初回起動時はimu resetスイッチを押して起動すること)
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            mpu.setOffset(&info.imuOffsets.acc, &info.imuOffsets.gyro);
            printf("ACC offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
            HAL_Delay(1000);
      }
      info.frontDeg = att.z;
}

void Robot::photoThresholdSet() {
      uint16_t thr = PHOTOSENSOR_THRESHOLD;
      CANBus::CANData data = {
          .stdId = PHOTOTHRESHOLD,
          .data = {
              (uint8_t)(thr & 0xFF),
              (uint8_t)((thr >> 8) & 0xFF),
          },
      };
      can.send(data);
}
