#ifndef _MODE_TEST_IMU_
#define _MODE_TEST_IMU_

#include "setup.h"

// #define TEST1
#define TEST2
Timer tim;
int16_t velAngular = 0;
void before_test_imu() {
    pc.printf("before test imu\r\n");
    imu.setZero();
    tim.start();
}

void body_test_imu() {
    info.imuDirPrev = info.imuDir;
    info.imuDir = imu.getDeg();
    velAngular = info.imuDir - info.imuDirPrev;
#ifdef TEST1
    if (tim.read_ms() < 1000) {
        pidDir.target = 0;
    } else if (tim.read_ms() < 2000) {
        pidDir.target = -120;
    } else if (tim.read_ms() < 3000) {
        pidDir.target = 60;
    } else {
        tim.reset();
    }
#endif
#ifdef TEST2
    if (tim.read_ms() < 2000) {
        imu.setDeg(0);
    } else if (tim.read_ms() < 2000) {
        imu.setDeg(60);
    } else {
        tim.reset();
    }
#endif
    pidDir.rawData = info.imuDir;

    int16_t m_power = getTurnAttitude();
    pc.printf("imu:%f %d %f V:%d\r\n", pidDir.currentData, m_power, pidDir.totalError, info.volt);
    MD.setMotors(info, m_power, m_power, m_power, m_power);
}

void after_test_imu() {
    pc.printf("after test imu\r\n");
    MD.setVelocityZero();
}

const RIMode modeTestIMU = {
    modeName : "mode_test_imu",
    modeLetter : 'I',
    before : callback(before_test_imu),
    body : callback(body_test_imu),
    after : callback(after_test_imu)
};

#endif