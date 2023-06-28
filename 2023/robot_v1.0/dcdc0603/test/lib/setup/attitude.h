#ifndef ATTITUDE_H
#define ATTITUDE_H
#include "mbed.h"

typedef struct {
    float rawData;  // コンパスのデータ
    float lastData; // 前回
    float currentData;
    float out;
    int16_t target;     // 目標点
    int16_t targetPrev; // 目標点
    float differential;

    float totalError;
    float Ki;
    float Kd;
    float Kp;

    float P;
    float I;
    float D;
    int16_t turn;
} type_pid;

Ticker tickCalcIMU;
Timer pidDt;
type_pid pidDir;

bool imuDirEnable = true;
int16_t degBetween(int16_t deg1, int16_t deg2) {
    int16_t a = deg1 - deg2;
    while (a < 0)
        a += 360;
    while (a > 180)
        a -= 360;
    return abs(a);
}

int16_t degBetween_signed(int16_t deg1, int16_t deg2) {
    int16_t a = deg1 - deg2;
    while (a < 0)
        a += 360;
    while (a > 180)
        a -= 360;
    return a;
}

void setTargetDir(int16_t target) {
    pidDir.target = target;
}

void setPIDGain() {
    pidDir.Kp = -0.6;
    pidDir.Kd = 0.05;
    pidDir.Ki = -0.35;
    // pidDir.Kp = -0.3;
    // pidDir.Kd = 0.05;
    // pidDir.Ki = 0;
}

void attitudeControl() {
    if (imuDirEnable) {
        pidDir.currentData = degBetween_signed(pidDir.rawData, pidDir.target);
        pidDir.differential = (pidDir.lastData - pidDir.currentData) / 0.01;
        pidDir.lastData = pidDir.currentData;

        if (abs(pidDir.currentData) < 30 && pidDir.target == pidDir.targetPrev) { // 目標からの誤差が大きい時に積分をすると発散してしまうので無理やり計算しないようにした
            if (pidDir.currentData != 0) {
                if (abs(pidDir.I) < 30) pidDir.totalError += pidDir.currentData * 0.01; // 無限に発散されると困るので上限をつけた
            } else {
                pidDir.totalError = 0; // 0の時に10とか残ってたりするからリセットかけるようにした
            }
        } else {
            pidDir.totalError = 0;
        }

        pidDir.P = pidDir.Kp * pidDir.currentData;
        pidDir.I = pidDir.Ki * pidDir.totalError;
        pidDir.D = pidDir.Kd * pidDir.differential;

        pidDir.out = pidDir.P + pidDir.I + pidDir.D;
        if (abs(pidDir.out) > 35) {
            pidDir.out = pidDir.out / abs(pidDir.out) * 35;
        }
        pidDir.targetPrev = pidDir.target;
        pidDir.turn = pidDir.out;
    }
}

int16_t getTurnAttitude() {
    return pidDir.turn;
}
#endif