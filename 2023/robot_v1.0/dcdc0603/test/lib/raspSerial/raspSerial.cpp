#include "raspSerial.h"

raspSerial::raspSerial(PinName TX, PinName RX, RawSerial *_pc) : device(TX, RX) {
    device.baud(115200);                                   // 通信速度最速
    device.attach(callback(this, &raspSerial::receiveRx)); // 角度割り込み入力
    pc = _pc;
    bufferCount = 0;
}

raspSerial::raspSerial(PinName TX, PinName RX, RawSerial *_pc, int baud) : device(TX, RX) {
    device.baud(baud);                                     // 通信速度最速
    device.attach(callback(this, &raspSerial::receiveRx)); // 角度割り込み入力
    pc = _pc;
    bufferCount = 0;
}

// 受信割り込み関数
void raspSerial::receiveRx() {
    // bufferCount++;
    // if (bufferCount > BUFFER_SIZE) bufferCount = 0;
    // recievedData = device.getc();
    // buffer[bufferCount - 1] = recievedData;
    bufferCount++;
    buffer[bufferCount - 1] = device.getc();
    if (buffer[0] != 0xFF) bufferCount--;
    if (bufferCount == BUFFER_SIZE) {
        // bufferCount == BUFFER_SIZE 全ての通信が完了した時に以下を実行
        if (buffer[0] == 0xFF) {
            bufferCount = 0;
            // 受信バッファに溜まったデータを実用の変数に代入していく
            // for使うより直接代入の方が早い
            info.motor[0] = (int16_t)buffer[1] - 100.0; //
            info.motor[1] = (int16_t)buffer[2] - 100.0;
            info.motor[2] = (int16_t)buffer[3] - 100.0;
            info.motor[3] = (int16_t)buffer[4] - 100.0;
            info.driblePower = (float)buffer[5] / 100;
            info.kickerPower[STRAIGHT_KICKER] = (float)buffer[6] / 100;
            info.kickerPower[CHIP_KICKER] = (float)buffer[7] / 100;
            // if (buffer[9] == 3) {
            //     info.imuTargetDir = 0;
            // } else {
            //     info.imuTargetDir = (float)((int16_t)(buffer[8] * (buffer[9] == 1 ? -1 : 1)));
            // }

            // byte[8]について(符号)
            // 0:1
            // 1:-1
            // 2:1
            // 3:-1

            info.imuTargetDir = (float)((int16_t)(buffer[8] * (buffer[9] % 2 == 1 ? -1 : 1)));

            info.imuStatus = buffer[9];
            info.emergency = buffer[10];
        } else {
            pc->printf("e_b[0] == 0xFF : %d ,bC = %d\r\n", recievedData, bufferCount);
            bufferCount = 0;
        }
    }

    // pc->printf("DATA%d:%d ", bufferCount, recievedData);
    //  if (buffer[0] != 0xFF) bufferCount--;
    // if (buffer[0] == 0xFF) {
    //     if (buffer[1] == 0x00) {
    //         if (buffer[2] == 0xFF) {
    //             if (buffer[3] == 0x00) {
    //                 //受信バッファに溜まったデータを実用の変数に代入していく
    //                 // for使うより直接代入の方が早い
    //                 // info.motor[0] = (int16_t)buffer[4] - 100.0; //
    //                 // info.motor[1] = (int16_t)buffer[5] - 100.0;
    //                 // info.motor[2] = (int16_t)buffer[6] - 100.0;
    //                 // info.motor[3] = (int16_t)buffer[7] - 100.0;
    //                 // info.driblePower = (float)buffer[8] / 100;
    //                 // info.kickerPower[STRAIGHT_KICKER] = (float)buffer[9] / 100;
    //                 // info.kickerPower[CHIP_KICKER] = (float)buffer[10] / 100;
    //                 // info.imuTargetDir = (float)((int16_t)(buffer[11] * 256 + buffer[12]));
    //                 // info.emergency = buffer[13];
    //             } else {
    //                 if (bufferCount == 3) return;
    //                 pc->printf("e_b[3] == 0x00 : %d ,bC = %d\r\n", recievedData, bufferCount);
    //                 bufferCount = 3;
    //             }
    //         } else {
    //             if (bufferCount == 2) return;
    //             pc->printf("e_b[2] == 0xFF : %d ,bC = %d\r\n", recievedData, bufferCount);
    //             bufferCount = 2;
    //         }
    //     } else {
    //         if (bufferCount == 1) return;
    //         pc->printf("e_b[1] == 0x00 : %d ,bC = %d\r\n", recievedData, bufferCount);
    //         bufferCount = 1;
    //     }
    // } else {
    //     pc->printf("e_b[0] == 0xFF : %d ,bC = %d\r\n", recievedData, bufferCount);
    //     bufferCount = 0;
    // }
}

void raspSerial::sendToRasp(RobotInfo info) {
    uint8_t buffer[6];
    char startBytes[4]{0xFF, 0, 0xFF, 0};
    buffer[0] = info.volt;
    buffer[1] = info.photoSensor >> 8;     // MSB
    buffer[2] = info.photoSensor & 0x00FF; // LSB
    buffer[3] = info.isHoldBall;
    buffer[4] = (int16_t)(info.imuDir) >> 8;     // MSB
    buffer[5] = (int16_t)(info.imuDir) & 0x00FF; // LSB
    // 送信バッファに溜まったデータを送信
    for (int i = 0; i < 4; i++) {
        device.putc(startBytes[i]);
        // wait_us(500);
    }
    for (int i = 0; i < 6; i++) {
        device.putc(buffer[i]);
        // wait_us(500);
    }
}

// void raspSerial::sendToRasp(RobotInfo info) {
//     uint8_t buffer[6];
//     uint8_t startBytes[4] {0xFF,0,0xFF,0};
//     buffer[0] = 10;
//     buffer[1] = 20;
//     buffer[2] = 30;
//     buffer[3] = 40;
//     buffer[4] = 50;
//     buffer[5] = 60;
//     //送信バッファに溜まったデータを送信
//     for (int i = 0; i < 4; i++) {
//         device.putc(startBytes[i]);
//     }
//     for (int i = 0; i < 6; i++) {
//         device.putc(buffer[i]);
//     }
// }

void raspSerial::put(int val) { device.putc(val); }
void raspSerial::get(float &a, int num) { a = info.motor[num]; }
void raspSerial::print(float val) { device.printf("%f\r\n", val); }
void raspSerial::syncFromRasp(RobotInfo &_info) { _info = info; }