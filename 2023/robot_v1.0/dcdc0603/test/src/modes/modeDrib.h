#ifndef _MODEDRIB_
#define _MODEDRIB_

#include "setup.h"

void before_drib() {
    // bodyを実行する直前に1度だけ実行する関数
    pc.printf("before driber test\r\n");
}

void body_drib() {
    // モードのメインプログラムを書く関数.この関数がループで実行されます
    // float p = 1.0;
    // dribler.write(p);

    getSensors(info);
    pc.printf("sens:%d", info.photoSensor);
    if (info.isHoldBall) {
        pc.printf(" HOLD!!!");
        // dribler.write(1.0);
        dribbler.setPower(1.0);
    } else {
        // dribler.write(0.3);
        dribbler.setPower(0.3);
    }
    dribbler.dribble();

    pc.printf("drib power:%f\r\n", dribbler.getPower());
}

void after_drib() {
    // モードが切り替わり、bodyが実行し終えた直後に1度だけ実行する関数
    pc.printf("after driber test\r\n");
    // dribler.write(0.0);
    dribbler.turnOff();
}

const RIMode modeDrib = {
    modeName :
        "mode_drib_test", // モードの名前.コンソールで出力したりLCDに出せます.
    modeLetter : 'D',     // モード実行のコマンド
    before : callback(before_drib),
    body : callback(body_drib),
    after : callback(after_drib)
};

#endif