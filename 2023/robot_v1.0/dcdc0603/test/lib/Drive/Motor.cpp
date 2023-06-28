#include "Motor.h"
#define MOTOR_TEST_IGNORE
Motor::Motor(PinName CAN_TX, PinName CAN_RX, PinName testSW)
    : canMBED(CAN_TX, CAN_RX), switch_1(testSW) {
    canMBED.frequency(100000);
}

void Motor::setMotors(RobotInfo &info, int16_t m0, int16_t m1, int16_t m2, int16_t m3) {
    info.motor[0] = m0;
    info.motor[1] = m1;
    info.motor[2] = m2;
    info.motor[3] = m3;
    motors.M1.vel = info.motor[0];
    motors.M2.vel = info.motor[1];
    motors.M3.vel = info.motor[2];
    motors.M4.vel = info.motor[3];
    sendMotorValues();
}

void Motor::setVelocity(RobotInfo &info, int8_t turn) {
#ifndef MOTOR_TEST_IGNORE
    motors.M1.vel = 20;
    motors.M2.vel = 20;
    motors.M3.vel = 20;
    motors.M4.vel = 20;
#else
    motors.M1.vel = info.motor[0] + turn;
    motors.M2.vel = info.motor[1] + turn;
    motors.M3.vel = info.motor[2] + turn;
    motors.M4.vel = info.motor[3] + turn;
#endif
    sendMotorValues();
}

void Motor::setVelocity(RobotInfo &info) {
#ifndef MOTOR_TEST_IGNORE
    motors.M1.vel = 20;
    motors.M2.vel = 20;
    motors.M3.vel = 20;
    motors.M4.vel = 20;
#else
    motors.M1.vel = info.motor[0];
    motors.M2.vel = info.motor[1];
    motors.M3.vel = info.motor[2];
    motors.M4.vel = info.motor[3];
#endif
    sendMotorValues();
}

void Motor::setVelocityZero() {
    motors.M1.vel = 0;
    motors.M2.vel = 0;
    motors.M3.vel = 0;
    motors.M4.vel = 0;
    sendMotorValues();
}

void Motor::sendMotorValues() {
    if (emergency == true) {
        motors.M1.vel = 0;
        motors.M2.vel = 0;
        motors.M3.vel = 0;
        motors.M4.vel = 0;
    }

    // int hosho = 8;
    // if (motors.M1.vel > 0 && motors.M1.vel < hosho) {
    //     motors.M1.vel = hosho;
    // } else if (motors.M1.vel < 0 && motors.M1.vel > -hosho) {
    //     motors.M1.vel = -hosho;
    // }
    // if (motors.M2.vel > 0 && motors.M2.vel < hosho) {
    //     motors.M2.vel = hosho;
    // } else if (motors.M2.vel < 0 && motors.M2.vel > -hosho) {
    //     motors.M1.vel = -hosho;
    // }
    // if (motors.M3.vel > 0 && motors.M3.vel < hosho) {
    //     motors.M3.vel = hosho;
    // } else if (motors.M3.vel < 0 && motors.M3.vel > -hosho) {
    //     motors.M1.vel = -hosho;
    // }
    // if (motors.M4.vel > 0 && motors.M4.vel < hosho) {
    //     motors.M4.vel = hosho;
    // } else if (motors.M4.vel < 0 && motors.M4.vel > -hosho) {
    //     motors.M1.vel = -hosho;
    // }
    send_motvel_data[0] = motors.M1.vel8_t.L;
    send_motvel_data[1] = motors.M1.vel8_t.H;
    send_motvel_data[2] = motors.M2.vel8_t.L;
    send_motvel_data[3] = motors.M2.vel8_t.H;
    send_motvel_data[4] = motors.M3.vel8_t.L;
    send_motvel_data[5] = motors.M3.vel8_t.H;
    send_motvel_data[6] = motors.M4.vel8_t.L;
    send_motvel_data[7] = motors.M4.vel8_t.H;
    canMBED.write(CANMessage(0x1AA, send_motvel_data, 8));
}

void Motor::setEmergency() {
    emergency = true;
}

void Motor::setForceUnlockEmergency() {
    emergency = false;
}