#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "mbed.h"
#include "RobotInfo.h"

class Motor {
  public:
    Motor(PinName CAN_TX, PinName CAN_RX, PinName testSW);
    void setMotors(RobotInfo &info, int16_t m0, int16_t m1, int16_t m2, int16_t m3);
    void setVelocity(RobotInfo &info, int8_t turn);
    void setVelocity(RobotInfo &info);
    void setVelocityZero();
    void sendMotorValues();
    void setEmergency();
    void setForceUnlockEmergency();

  private:
    typedef union {
        int16_t vel; // main value
        struct {
            char L : 8; // LOW byte
            char H : 8; // High byte
        } vel8_t;
    } moterOrder;

    typedef struct {
        moterOrder M1;
        moterOrder M2;
        moterOrder M3;
        moterOrder M4;
    } motorsVel;

    motorsVel motors;
    char send_motvel_data[8];
    CAN canMBED;
    DigitalIn switch_1;
    bool emergency;
};

#endif