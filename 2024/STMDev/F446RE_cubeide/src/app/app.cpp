#include "app.h"

#include "main.h"
#include "DMAStream.h"
#include "Timer.hpp"
#include "MyMath.hpp"
#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "BufferedSerial.hpp"
#include "CAN.hpp"
#include "BNO055.hpp"

#include "adc.h"

#include <Mode.hpp>

CANBus can(&hcan1, 0x100);
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);
DigitalOut led1(LED1_GPIO_Port, LED1_Pin);
DigitalOut led2(LED2_GPIO_Port, LED2_Pin);
PwmSingleOut ledH(&htim1, TIM_CHANNEL_1);
BufferedSerial serial1(&huart1, 128); // pc
BufferedSerial serial4(&huart4, 128); // xiao
BufferedSerial serial5(&huart5, 256); // RasPi
BNO055 bno(&hi2c1);

typedef struct {
    //     /*
    //     0. ヘッダ0xFFを受信
    //     1. velX[LSB]
    //     2. velX[MSB]
    //     3. velY[LSB]
    //     4. velY[MSB]
    //     5. velAngler[LSB]
    //     6. velAngler[MSB]
    //     7. dribblePower[0~100]
    //     8. kickerPowerStraight[0~100]
    //     9. kickerPowerChip[0~100]
    //     10.status
    //       . bit0: emergencyStop
    //       . bit1: doDirectKick
    //       . bit2: doDirectChipKick
    //       . bit3: reserved
    //       . bit4: reserved
    //       . bit5: reserved
    //       . bit6: reserved
    //       . bit7: parity
    //      */

    // Infomation RaspberryPi→STM32
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velX;
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velY;
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velAngler;

    uint8_t dribblePower;
    uint8_t kickerPowerStraight;
    uint8_t kickerPowerChip;
    union {
        struct {
            bool emergencyStop : 1;
            bool doDirectKick : 1;
            bool doDirectChipKick : 1;
            bool reserved0 : 1;
            bool reserved2 : 1;
            bool reserved3 : 1;
            bool reserved4 : 1;
            bool parity : 1;
        };
        uint8_t data;
    } status;

    // Infomation STM32→RaspberryPi
    uint8_t photoSensorValue;
    uint8_t isHoldBall;
    uint8_t batteryVoltage;
} RobotInfo;

CANBus::CANData canRecvData;

Timer raspSendTimer;
Timer mdSendTimer;
Timer chargeTimer;

uint8_t photoSensorValue = 0;
CANBus::CANData canSend_Date;

RobotInfo info;

inline __attribute__((always_inline)) void heartBeat() {
    static int i = 0;
    i++;
    ledH.write(MyMath::sinDeg(int(i / 5)) / 2 + 0.5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim10) {
        heartBeat(); // 1ms
    } else {
        // pass
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
        led0 = !led0;

        switch (canRecvData.stdId) {
        case 0x123: // フォトセンサの値
            photoSensorValue = canRecvData.data[0];
            break;

        default:
            break;
        }
    }
}

void rasRecvSerial() {

    // 0. ヘッダ0xFFを受信
    // 1. velX[LSB]
    // 2. velX[MSB]
    // 3. velY[LSB]
    // 4. velY[MSB]
    // 5. velAngler[LSB]
    // 6. velAngler[MSB]
    // 7. dribblePower[0~100]
    // 8. kickerPowerStraight[0~100]
    // 9. kickerPowerChip[0~100]
    // 10.status
    //   . bit0: emergencyStop
    //   . bit1: doDirectKick
    //   . bit2: doDirectChipKick
    //   . bit3: reserved
    //   . bit4: reserved
    //   . bit5: reserved
    //   . bit6: reserved
    //   . bit7: parity

    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 10;  // データのサイズ
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
                    info.kickerPowerStraight = data[7];
                    info.kickerPowerChip = data[8];
                    info.status.data = data[9];

                    headerReceived = false; // 次のヘッダを待つ準備をする
                    index = 0;              // インデックスをリセット
                }
            }
        }
    }
}

void rasSendSerial(RobotInfo &info) {
    static const uint8_t dataSize = 4; // データのサイズ
    uint8_t startBytes[4] = {0xFF, 0, 0xFF, 0};

    uint8_t buffer[dataSize] = {
        info.batteryVoltage,
        info.photoSensorValue & 0x00FF,
        info.photoSensorValue >> 8,
        info.isHoldBall,
    };
    serial5.write(startBytes, 4);
    serial5.write(buffer, dataSize);
}

void setup() {
    for (size_t i = 0; i < 10; i++) {
        led1 = bno.check();
        HAL_Delay(50);
    }
    bno.setUnit(1, 1, 1, 0);
    bno.setPowerMode();
    // bno.setOperaitonMode(OPERATION_MODE_AMG);
    bno.setOperaitonMode(OPERATION_MODE_NDOF);
    // bno.accConfig();
    bno.init();
    can.init();

    serial1.init();
    serial4.init();
    serial5.init();
    ledH.init();
    printf("Hello World\n");

    // setVelocityZero();
    raspSendTimer.reset();
    mdSendTimer.reset();
    chargeTimer.reset();

    bno.setAttitudeZero();

    wait_ms(1000);
    // sendBoosterChargeStart();

    led0 = 0;
}

void main_app() {
    setup();
    while (1) {

        MyMath::printBit(info.status.data);
        // printBit(10);
    }
}