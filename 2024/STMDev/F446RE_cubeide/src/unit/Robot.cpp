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

    // bno.setAttitudeZero();
    // HAL_Delay(1000);
}

void Robot::rasRecvSerial() {
    // 0. ヘッダ0xFFを受信
    // 1. velX[LSB]
    // 2. velX[MSB]
    // 3. velY[LSB]
    // 4. velY[MSB]
    // 5. velAngler[LSB]
    // 6. velAngler[MSB]
    // 7. dribblePower[0~100] [uint8_t]
    // 8. kickerPowerStraight[0~100] [uint8_t]
    // 9. kickerPowerChip[0~100] [uint8_t]
    // 10. relativePositionX[LSB]
    // 11. relativePositionX[MSB]
    // 12. relativePositionY[LSB]
    // 13. relativePositionY[MSB]
    // 14. relativeTheta[LSB]
    // 15. relativeTheta[MSB]
    // 16. cameraX [uint8_t]
    // 17. cameraY [uint8_t]
    // 18.status
    //   . bit0: emergencyStop
    //   . bit1: doDirectKick
    //   . bit2: doDirectChipKick
    //   . bit3: reserved
    //   . bit4: reserved
    //   . bit5: reserved
    //   . bit6: isCtrlByRobot (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
    //   . bit7: parity

    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 18;  // データのサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[dataSize] = {0}; // 受信したデータ

    while (serial5.available()) {
        // 1バイト読み込み
        uint8_t receivedByte = serial5.read();
        // printf("received %d\n ", receivedByte);

        if (!headerReceived) {
            index = 0;
            if (receivedByte == HEADER) {
                // ヘッダを受信したらデータの受信を開始
                headerReceived = true; // ヘッダを受信したフラグを立てる
                // printf("header received %d\n ", receivedByte);
            } else {
                headerReceived = false; // ヘッダではないのでフラグをリセット
                // printf("error: Header is not received %d\n", receivedByte);
            }
        } else { // ヘッダを受信した後の処理
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

                    headerReceived = false; // 次のヘッダを待つ準備をする
                    index = 0;              // インデックスをリセット
                }
            }
        }
    }
}

void Robot::rasSendSerial(RobotInfo &info, uint16_t interval) {
    static const uint8_t dataSize = 3; // データのサイズ
    static const uint8_t startBytes[4] = {0xFF, 0, 0xFF, 0};

    if (rasSendInterval.read_ms() < interval) {
        return;
    }
    uint8_t buffer[dataSize] = {
        info.batteryVoltage,
        info.isHoldBall,
        info.capChargeCertitude,
    };
    serial5.write(startBytes, 4);
    serial5.write(buffer, dataSize);
    rasSendInterval.reset();
}

void Robot::getSensors(RobotInfo *info) {
    static uint16_t underVoltageCount = 0;
    // バッテリー電圧
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t batt = HAL_ADC_GetValue(&hadc1);
    info->batteryVoltage = medianBatteryValue.calc((uint8_t)((float)(batt) * 3.3 / 4095.0 * 58.5 - 7));
    info->isHoldBall = (medianPhotoValue.calc(info->photoSensorValue) < PHOTOSENSOR_THRESHOLD); // config
    info->capChargeCertitude = getCapChargeCertitude();
    if (info->batteryVoltage < BATTERY_CUT_OFF) {
        if (underVoltageCount < 1000) underVoltageCount++;
    } else {
        if (underVoltageCount > 0) underVoltageCount--;
    }
    info->isUnderVoltage = (underVoltageCount == 0);
}
