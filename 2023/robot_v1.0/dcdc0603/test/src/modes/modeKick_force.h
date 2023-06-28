#ifndef _MODEKICK_FORCE_
#define _MODEKICK_FORCE_

#include "setup.h"

void before_kick_force() { pc.printf("before Kicker Test\r\n"); }

// KICKER_STRAIGHTでストレートキック
// CHIP_KICKERでチップキック

void body_kick_force() {
    float p = 1.0;
    getSensors(info);
    pc.printf("sens:%d", info.photoSensor);
    pc.printf("FORCE KICK!!!(STRAIGHT)\r\n");
    wait_ms(2000);
    pc.printf("3\r\n");
    wait(1);
    pc.printf("2\r\n");
    wait(1);
    pc.printf("1\r\n");
    wait(1);
    pc.printf("STRAIGHT KICK!!! \r\n");

    kicker[STRAIGHT_KICKER].setPower(p); // power:0.0~1.0
    kicker[STRAIGHT_KICKER].Kick();

    for (size_t i = 0; i < 100; i++) {
        pc.printf("waiting... %d\r\n", i);
        wait_ms(100);
    }

    pc.printf("FORCE KICK!!!(CHIP)\r\n\r\n");
    wait_ms(2000);
    pc.printf("3\r\n");
    wait(1);
    pc.printf("2\r\n");
    wait(1);
    pc.printf("1\r\n");
    wait(1);
    pc.printf("CHIP KICK!!! \r\n");
    kicker[CHIP_KICKER].setPower(p); // power:0.0~1.0
    kicker[CHIP_KICKER].Kick();

    while (mode != 'M') {
        pc.printf("Please send 'M' to quit... \r\n");
        wait(1);
    }
}

void after_kick_force() {
    pc.printf("after FORCE Kicker Test\r\n");
    kicker[STRAIGHT_KICKER].setPower(0);
    kicker[CHIP_KICKER].setPower(0);
}

const RIMode modeKickForce = {
    modeName : "mode_kick_test_force", //モードの名前.コンソールで出力したりLCDに出せます.
    modeLetter : 'F',                  //モード実行のコマンド
    before : callback(before_kick_force),
    body : callback(body_kick_force),
    after : callback(after_kick_force)
};

#endif