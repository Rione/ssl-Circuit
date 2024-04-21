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

CANBus can(&hcan1, 0x100);
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);
DigitalOut led1(LED1_GPIO_Port, LED1_Pin);
DigitalOut led2(LED2_GPIO_Port, LED2_Pin);
PwmSingleOut ledH(&htim1, TIM_CHANNEL_1);
BufferedSerial serial1(&huart1, 128);
BufferedSerial serial4(&huart4, 128);
BufferedSerial serial5(&huart5, 128);
BNO055 bno(&hi2c1);

CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {100, 200, 0, 0, 0, 0, 0, 8},
};
Timer timer;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == &hcan1) {
        can.recv(canRecvData);
        canRecvData.stdId = 0x555;
        can.send(canRecvData);
        led0 = !led0;
    }
}

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

// シリアルで受信したデータを処理するサンプル
uint8_t dataFlame[8] = {0};
void processPacket() {
    /*
    0. ヘッダ0xAAを受信
    1.
    2.
    3.
    4.
    5.
    6.
    7.
    8.
    9. チェックサムを受信
     */
    const uint8_t HEADER = 0xAA;        // 仮のヘッダ
    const uint8_t dataSize = 8;         // データのサイズ
    static bool headerReceived = false; // ヘッダを受信したかどうか
    static uint8_t sum = 0;             // チェックサム計算用
    static uint8_t index = 0;           // 受信したデータのインデックスカウンター
    static int8_t data[dataSize] = {0}; // 受信したデータ

    while (serial1.available()) {
        // 1バイト読み込み
        uint8_t receivedByte = serial1.read();

        printf("headerReceived:%d, index:%d receivedByte:%d\n", headerReceived, index, receivedByte);
        if (!headerReceived) {
            index = 0;
            if (receivedByte == HEADER) {
                // ヘッダを受信したらデータの受信を開始
                headerReceived = true; // ヘッダを受信したフラグを立てる
                printf("header received %d\n ", receivedByte);
            } else {
                headerReceived = false; // ヘッダではないのでフラグをリセット
                printf("error: Header is not received %d\n", receivedByte);
            }
        } else { // ヘッダを受信した後の処理
            if (index < dataSize) {
                // データ受信
                data[index] = receivedByte;
                index++;
                // checksum
                sum += receivedByte;
            } else { // データ受信完了
                // チェックサムの処理
                uint8_t checksum = receivedByte;
                if (sum == checksum) {
                    // 受信成功
                    printf("success: Check sum match %d == %d\n", sum, checksum);
                    printf("data:{%3d %3d %3d %3d %3d %3d %3d %3d}\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                    memcpy(dataFlame, data, dataSize); // 受信データをコピー
                } else {
                    // 受信失敗
                    printf("error: Check sum doesn't match %d != %d\n", sum, checksum);
                }
                headerReceived = false; // 次のヘッダを待つ準備をする
            }
        }
    }
}

void setup() {
    bno.check();
    bno.setUnit(1, 1, 1, 0);
    bno.setPowerMode();
    bno.setOperaitonMode(OPERATION_MODE_AMG);
    bno.accConfig();
    bno.init();
    can.init();
    serial1.init();
    serial4.init();
    serial5.init();
    ledH.init();
    printf("Hello World\n");
}

void main_app() {
    setup();
    while (1) {
        if (serial4.available()) {
            uint8_t data = serial4.read();
            printfDMA("recive:%c\n", data); // なぜかDMAStreamを使わないとprintfが使えない
            led0 = !led0;
        }
        HAL_Delay(100);
        // printf("Hello World\n");
    }
}