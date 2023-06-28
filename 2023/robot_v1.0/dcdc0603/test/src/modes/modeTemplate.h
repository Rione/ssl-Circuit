#ifndef _MODE_TEMP_
#define _MODE_TEMP_

#include "setup.h"

void before_temp() {
    // bodyを実行する直前に1度だけ実行する関数
    pc.printf("before temp\r\n");
}

void body_temp() {
    // モードのメインプログラムを書く関数.この関数がループで実行されます
    pc.printf("body temp\r\n");
}

void after_temp() {
    // モードが切り替わり、bodyが実行し終えた直後に1度だけ実行する関数
    pc.printf("after temp\r\n");
}

const RIMode modetemp = {
    modeName : "mode_temp", //モードの名前.コンソールで出力したりLCDに出せます.
    modeLetter : 'T', //モード実行のコマンド
    before : callback(before_temp),
    body : callback(body_temp),
    after : callback(after_temp)
};

#endif