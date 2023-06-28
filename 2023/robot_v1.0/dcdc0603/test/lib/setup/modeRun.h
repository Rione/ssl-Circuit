#ifndef _MODERUN_
#define _MODERUN_

#include "setup.h"
#include "../src/modes/modes.h"

RobotInfo info = {0};
int16_t emergencyCounter = 0;
// パソコンからシリアル受信でモードコマンドを取得する関数
void receiveCommand() {
    mode = pc.getc();
    pc.printf("receive:%c\r\n", mode);
    if (mode == 'U') {
        MD.setForceUnlockEmergency();
        pc.printf("\r\n-------------------------------------------- \r\n UNLOCK EMERAGENCY MODE RISK{Over discharge} V:%d\r\n-------------------------------------------- \r\n", info.volt);
    } else if (mode == 's') {
        MD.setEmergency();
        MD.setVelocityZero();
        pc.printf("\r\n STOP!!!!!! \r\n");
    }
}

// modes.hの中にあるmodesリストとPCから送られてきたコマンドを比較して検証・変換する関数
int8_t checkModeMatch(char &m) {
    for (size_t i = 0; i < modeIndex; i++) {
        if (modes[i].modeLetter == m) {
            return i;
        }
    }
    return MODE_UNMATCH;
}

void checkBattery(RobotInfo &info) {
    if (info.volt < BATTERY_THRESHOLD) {
        emergencyCounter++;
    } else {
        emergencyCounter--;
    }
    if (emergencyCounter >= 300) {
        MD.setEmergency();
        MD.setVelocityZero();
        pc.printf("RISK!!!{Over discharge} V:%d ", info.volt);
        emergencyCounter = 300;
    } else if (emergencyCounter <= 0) {
        emergencyCounter = 0;
    }
}

void initModeRun() {
    mode = 'M'; //デフォルトはmodeMainで始めます
    pc.attach(&receiveCommand, Serial::RxIrq);

    pc.printf("Boot!!\r\n");
    pc.printf("\r\nHello world!! Ri-one SSL\r\n");
    pc.printf("STM32 works with %ld MHz clock\r\n", (SystemCoreClock / 1000000));
    wait_ms(100);
    pc.printf("-----------------------------------------------\r\n");
    pc.printf("This Robots has %d modes!!\r\n", modeIndex);
    for (size_t i = 0; i < modeIndex; i++) {
        pc.printf(" -- %c : %s\r\n", modes[i].modeLetter, modes[i].modeName);
    }
    pc.printf("To run each modes, Please Type Letter like '%c'.\r\n", modes[0].modeLetter);
    pc.printf("-----------------------------------------------\r\n");
    wait_ms(2000);
    target = modes[0];
    targetPrev = modes[0];
}

void modeRun() {
    timer.reset();
    getSensors(info);
    checkBattery(info);
    runningModeIndex = checkModeMatch(mode);
    if (runningModeIndex != MODE_UNMATCH) {
        target = modes[runningModeIndex];
        targetPrev = modes[runningModeIndexPrev];
        if (runningModeIndexPrev == runningModeIndex) {
            target.body(); //モードのメインプログラムの実行
        } else {
            targetPrev.after();
            target.before();
        }
    } else {
        pc.printf("Couldn't find a mode on letter:%c\r\n", mode);
        mode = modes[runningModeIndexPrev].modeLetter;
        runningModeIndex = runningModeIndexPrev;
        pc.printf("-----------------------------------------------\r\n");
        pc.printf("This Robots has %d modes!!\r\n", modeIndex);
        for (size_t i = 0; i < modeIndex; i++) {
            pc.printf(" -- %c : %s\r\n", modes[i].modeLetter, modes[i].modeName);
        }
        pc.printf("-----------------------------------------------\r\n");
        wait_ms(500);
    }
    runningModeIndexPrev = runningModeIndex;
}

#endif