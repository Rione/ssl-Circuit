#ifndef _MODETEST_
#define _MODETEST_
#define MOTORTEST
#include "setup.h"

void before_test() {
    // bodyを実行する直前に1度だけ実行する関数
    pc.printf("before test\r\n");
    imu.setZero();
    // imu.setDeg(0);
}
// モードのメインプログラムを書く関数.この関数がループで実行されます

#ifndef MOTORTEST
void body_test() {
    int16_t m_turn = 0;
    actuatorTests();
    getSensors(info);
    if (IMU_CALIBURATION) {
        imu.setDeg(info.imuTargetDir);
    }
    if (!info.emergency) {
        // MD.setMotors(info,0,0,0,0);//motorのpower
        pidDir.target = info.imuTargetDir;
        pidDir.rawData = info.imuDir;
        m_turn = getTurnAttitude();
        if (IMU_CALIBURATION) {
            MD.setVelocityZero();
        } else {
            MD.setVelocity(info, m_turn);
        }
        if (info.kickerPower[STRAIGHT_KICKER] > 0) {
            kicker[STRAIGHT_KICKER].setPower(info.kickerPower[STRAIGHT_KICKER]);
            kicker[STRAIGHT_KICKER].Kick();
        }
        if (info.kickerPower[CHIP_KICKER] > 0) {
            kicker[CHIP_KICKER].setPower(info.kickerPower[CHIP_KICKER]);
            kicker[CHIP_KICKER].Kick();
        }
        dribler.write(info.driblePower); // power:0.0~1.0
    } else {
        pc.printf("emergency!!!\t");
        MD.setVelocityZero();
        kicker[STRAIGHT_KICKER].setPower(0.0);
        kicker[CHIP_KICKER].setPower(0.0);
        dribler.write(0);
    }
    rasp.sendToRasp(info);
    // pc.printf("%dus\r\n", timer.read_us());
    pc.printf("M1:%d\tM2:%d\tM3:%d\tM4:%d\tdrib:%.2f\tstraight:%.2f\tchip:"
              "%.2f\tvolt:%d\tPhoto:%d\timu:%.02f\ttargetDeg:%02f\temg:%d\tinterval:%dus\r\n",
              info.motor[0], info.motor[1], info.motor[2], info.motor[3],
              info.driblePower, info.kickerPower[STRAIGHT_KICKER],
              info.kickerPower[CHIP_KICKER], info.volt, info.photoSensor,
              info.imuDir, info.imuTargetDir, info.emergency, timer.read_us());
    // pc.printf("trueIMU:%.2f\t Dir:%.2f\t target:%.2f\t status:%d\r\n", imu.euler.yaw, info.imuDir, info.imuTargetDir, info.imuStatus);
}

#endif

#ifdef MOTORTEST
int count = 0;
int amout = 1;
int8_t p;
void body_test() {
    p = 0;
    amout = 1;
    pc.printf("MD0\r\n");
    for (int i = 0; i < 200; i++) {
        p += amout;
        if (p <= -100 || p >= 100) {
            amout = -amout;
        }
        MD.setMotors(info, p, 0, 0, 0);
        wait_ms(10);
    }
    p = 0;
    amout = 1;
    pc.printf("MD1\r\n");
    for (int i = 0; i < 200; i++) {
        p += amout;
        if (p <= -100 || p >= 100) {
            amout = -amout;
        }
        MD.setMotors(info, 0, p, 0, 0);
        wait_ms(10);
    }
    p = 0;
    amout = 1;
    pc.printf("MD2\r\n");
    for (int i = 0; i < 200; i++) {
        p += amout;
        if (p <= -100 || p >= 100) {
            amout = -amout;
        }
        MD.setMotors(info, 0, 0, p, 0);
        wait_ms(10);
    }
    p = 0;
    amout = 1;
    pc.printf("MD3\r\n");
    for (int i = 0; i < 200; i++) {
        p += amout;
        if (p <= -100 || p >= 100) {
            amout = -amout;
        }
        MD.setMotors(info, 0, 0, 0, p);
        wait_ms(10);
    }
}

#endif

void after_test() {
    // モードが切り替わり、bodyが実行し終えた直後に1度だけ実行する関数
    pc.printf("after test\r\n");
    // dribler.write(0);
    dribbler.turnOff();
    MD.setVelocityZero();
}

const RIMode modeTest = {
    modeName : "mode_test", // モードの名前.コンソールで出力したりLCDに出せます.
    modeLetter : 'T',       // モード実行のコマンド
    before : callback(before_test),
    body : callback(body_test),
    after : callback(after_test)
};

#endif