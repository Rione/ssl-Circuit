
#ifndef _RIHARDWARE_API_
#define _RIHARDWARE_API_

// Libs
#include "Servo.h"
#include "LGKicker.h"
#include "Dribbler.h"
#include "raspSerial.h"
#include "Motor.h"
#include "RobotInfo.h"
#include "LGIMU.h"

#include "pinDefs.h"
#include "setup_common.h"
#include "mbed.h"
#include "RIMode.h"

#define BALL_DETECT_VALUE 700
#define STRAIGHT_KICKER 0
#define CHIP_KICKER 1
#define BATTERY_THRESHOLD 132 // 13.2V

// Serial
asm(".global _printf_float"); // enables float print
RawSerial pc(USBTX, USBRX, 2000000);
raspSerial rasp(RASP_TX, RASP_RX, &pc, 9600);

Motor MD(CAN_TX, CAN_RX, MOTOR_TEST_SW);
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(&i2c);

// signals
AnalogIn ballPhoto(BALL_PHOTOSENS);
DigitalOut raspBallDetectSig(RASPI_SIG);
DigitalOut LED(LED1);

LGKicker kicker[2] = {KICKER_STRAIGHT, KICKER_CHIP};
Dribbler dribbler(DRIB_PWM);
// Servo dribler(DRIB_PWM);

// for test
DigitalIn swDrible(TEST_DRIB);
DigitalIn swKicker(TEST_KICK);

DigitalIn _voltIn(VOLT_IN, PullDown);
AnalogIn voltIn(VOLT_IN);

Timeout dribTimeout;
// Write Hardware API Fuctions under hear
uint8_t readBatteryVoltage() {
    uint16_t v_d = voltIn.read_u16();
    return (4.9E-09 * v_d * v_d + 0.0028 * v_d - 33);
}

void getSensors(RobotInfo &info) {

    rasp.syncFromRasp(info);
    info.photoSensor = ballPhoto.read_u16() / 65.535; // 1000分率に変換
    info.isHoldBall = (info.photoSensor < BALL_DETECT_VALUE);
    raspBallDetectSig = LED = info.isHoldBall;
    info.imuDirPrev = info.imuDir;
    info.imuDir = imu.getDeg();
    info.volt = readBatteryVoltage();
}

void dribleOff() {
    // タイマー割り込みでドリブルをオフにする
    // dribler.write(0.0);
    dribbler.turnOff();
}

void actuatorTests() {
    // dribler test
    if (swDrible.read() == false) {
        dribTimeout.attach(dribleOff, 1);
        // dribler.write(1.0);
        dribbler.setPower(1.0);
        dribbler.dribble();
    }
    // kicker test
    if (swKicker.read() == false) {
        kicker[STRAIGHT_KICKER].Kick();
    }
}

#endif