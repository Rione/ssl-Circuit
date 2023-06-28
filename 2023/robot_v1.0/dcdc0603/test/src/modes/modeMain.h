#ifndef _MODEMAIN_
#define _MODEMAIN_

#include "setup.h"

void before_main() {
    // bodyを実行する直前に1度だけ実行する関数
    pc.printf("before main\r\n");
    imu.setZero();
}
// モードのメインプログラムを書く関数.この関数がループで実行されます
void body_main() {

    int16_t m_turn = 0;
    bool isKick = false;
    actuatorTests();
    getSensors(info);
    if (!swKicker) {
        kicker[STRAIGHT_KICKER].discharge();
        pc.printf("discharge!!!\n");
    }
    if (!swDrible) {
        dribbler.dribble();
    }
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
        // dribler.write(info.driblePower); // power:0.0~1.0
        if (info.isHoldBall) {
            dribbler.setPower(info.driblePower);
        } else {
            if (info.driblePower != 0) {
                dribbler.setPower(0.3);
            } else {
                dribbler.setPower(0.0);
            }
        }
        if (info.kickerPower[STRAIGHT_KICKER] > 1.5 || info.kickerPower[CHIP_KICKER] > 1.5) {
            if (info.isHoldBall) {
                if (info.kickerPower[STRAIGHT_KICKER] > 0) {
                    isKick = true;
                    kicker[STRAIGHT_KICKER].setPower(info.kickerPower[STRAIGHT_KICKER] - 1.0);
                    kicker[STRAIGHT_KICKER].Kick();
                }
                if (info.kickerPower[CHIP_KICKER] > 0) {
                    isKick = true;
                    kicker[CHIP_KICKER].setPower(info.kickerPower[CHIP_KICKER] - 1.0);
                    kicker[CHIP_KICKER].Kick();
                }
                if (isKick == true) {
                    dribbler.turnOff();
                } else {
                    dribbler.dribble();
                }
            }
        } else {
            if (info.kickerPower[STRAIGHT_KICKER] > 0) {
                isKick = true;
                kicker[STRAIGHT_KICKER].setPower(info.kickerPower[STRAIGHT_KICKER]);
                kicker[STRAIGHT_KICKER].Kick();
            }
            if (info.kickerPower[CHIP_KICKER] > 0) {
                isKick = true;
                kicker[CHIP_KICKER].setPower(info.kickerPower[CHIP_KICKER]);
                kicker[CHIP_KICKER].Kick();
            }
            if (isKick == true) {
                dribbler.turnOff();
            } else {
                dribbler.dribble();
            }
        }

    } else {
        pc.printf("emergency!!!\t");
        MD.setVelocityZero();
        kicker[STRAIGHT_KICKER].setPower(0.0);
        kicker[CHIP_KICKER].setPower(0.0);
        // dribler.write(0);
        dribbler.turnOff();
    }
    rasp.sendToRasp(info);
    pc.printf("%dus\r\n", timer.read_us());
    // pc.printf("M1:%d\tM2:%d\tM3:%d\tM4:%d\tdrib:%.2f\tstraight:%.2f\tchip:"
    //           "%.2f\tvolt:%d\tPhoto:%d\timu:%.02f\ttargetDeg:%02f\temg:%d\tinterval:%dus\r\n",
    //           info.motor[0], info.motor[1], info.motor[2], info.motor[3],
    //           info.driblePower, info.kickerPower[STRAIGHT_KICKER],
    //           info.kickerPower[CHIP_KICKER], info.volt, info.photoSensor,
    //           info.imuDir, info.imuTargetDir, info.emergency, timer.read_us());
}

void after_main() {
    // モードが切り替わり、bodyが実行し終えた直後に1度だけ実行する関数
    pc.printf("after main\r\n");
    // dribler.write(0);
    dribbler.turnOff();
    MD.setVelocityZero();
}

// モード登録
const RIMode modeMain = {
    modeName : "mode_main", // モードの名前.コンソールで出力したりLCDに出せます.
    modeLetter : 'M',       // モード実行のコマンド
    before : callback(before_main),
    body : callback(body_main),
    after : callback(after_main)
};

#endif